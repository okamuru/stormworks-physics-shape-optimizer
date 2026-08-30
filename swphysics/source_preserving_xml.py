"""Losslessly reorder Stormworks vehicle components in the source XML bytes.

Stormworks writes compact vehicle XML.  Parsing and serializing the whole tree
with :mod:`xml.etree.ElementTree` changes otherwise unrelated spelling and, if
pretty-printing is enabled, can make a large vehicle tens of percent bigger.
This module therefore treats the XML parser as the semantic validator and uses
a small byte lexer only to locate direct ``<c>`` children of each vehicle body.
The original component byte ranges are moved between the original whitespace
slots; every other byte remains untouched.
"""

from __future__ import annotations

from array import array
from dataclasses import dataclass
import mmap
import os
from pathlib import Path
from typing import BinaryIO, List, Optional, Sequence, Tuple


_XML_WHITESPACE = frozenset(b" \t\r\n")


class SourceXmlLayoutError(ValueError):
    """The raw XML layout cannot be reordered without guessing."""


@dataclass
class _ComponentsSection:
    body_index: int
    content_start: int
    content_end: int
    component_starts: array
    component_ends: array


@dataclass
class _ElementFrame:
    name: bytes
    vehicle_body_index: Optional[int] = None
    components_section: Optional[_ComponentsSection] = None
    direct_component_start: Optional[int] = None


def _only_xml_whitespace(data: mmap.mmap, start: int, end: int) -> bool:
    return all(data[index] in _XML_WHITESPACE for index in range(start, end))


def _starts_with(data: mmap.mmap, marker: bytes, start: int) -> bool:
    return data[start : start + len(marker)] == marker


def _find_marker_end(
    data: mmap.mmap,
    start: int,
    marker: bytes,
    description: str,
) -> int:
    marker_start = data.find(marker, start)
    if marker_start < 0:
        raise SourceXmlLayoutError("unterminated {}".format(description))
    return marker_start + len(marker)


def _find_tag_end(data: mmap.mmap, start: int) -> int:
    quote = 0
    for index in range(start + 1, len(data)):
        value = data[index]
        if quote:
            if value == quote:
                quote = 0
        elif value in (ord('"'), ord("'")):
            quote = value
        elif value == ord(">"):
            return index + 1
    raise SourceXmlLayoutError("unterminated XML tag")


def _find_declaration_end(data: mmap.mmap, start: int) -> int:
    """Find ``>`` outside quotes and a DOCTYPE internal subset."""

    quote = 0
    bracket_depth = 0
    for index in range(start + 2, len(data)):
        value = data[index]
        if quote:
            if value == quote:
                quote = 0
            continue
        if value in (ord('"'), ord("'")):
            quote = value
        elif value == ord("["):
            bracket_depth += 1
        elif value == ord("]") and bracket_depth:
            bracket_depth -= 1
        elif value == ord(">") and bracket_depth == 0:
            return index + 1
    raise SourceXmlLayoutError("unterminated XML declaration")


def _normal_tag(
    data: mmap.mmap,
    start: int,
) -> Tuple[int, bytes, bool, bool]:
    """Return ``(end, name, closing, self_closing)`` for a normal tag."""

    end = _find_tag_end(data, start)
    cursor = start + 1
    while cursor < end - 1 and data[cursor] in _XML_WHITESPACE:
        cursor += 1
    closing = cursor < end - 1 and data[cursor] == ord("/")
    if closing:
        cursor += 1
        while cursor < end - 1 and data[cursor] in _XML_WHITESPACE:
            cursor += 1
    name_start = cursor
    while (
        cursor < end - 1
        and data[cursor] not in _XML_WHITESPACE
        and data[cursor] not in (ord("/"), ord(">"))
    ):
        cursor += 1
    if cursor == name_start:
        raise SourceXmlLayoutError("XML tag has no name at byte {}".format(start))
    name = data[name_start:cursor]
    trailer = end - 2
    while trailer > start and data[trailer] in _XML_WHITESPACE:
        trailer -= 1
    self_closing = not closing and data[trailer] == ord("/")
    return end, name, closing, self_closing


def _scan_vehicle_components(
    data: mmap.mmap,
) -> Tuple[Optional[_ComponentsSection], ...]:
    stack: List[_ElementFrame] = []
    body_sections: List[Optional[_ComponentsSection]] = []
    cursor = 0

    while cursor < len(data):
        tag_start = data.find(b"<", cursor)
        if tag_start < 0:
            if stack and stack[-1].components_section is not None:
                if not _only_xml_whitespace(data, cursor, len(data)):
                    raise SourceXmlLayoutError(
                        "non-whitespace text appears directly inside <components>"
                    )
            cursor = len(data)
            break

        if stack and stack[-1].components_section is not None:
            if not _only_xml_whitespace(data, cursor, tag_start):
                raise SourceXmlLayoutError(
                    "non-whitespace text appears directly inside <components>"
                )

        if _starts_with(data, b"<!--", tag_start):
            cursor = _find_marker_end(data, tag_start + 4, b"-->", "XML comment")
            continue
        if _starts_with(data, b"<![CDATA[", tag_start):
            end = _find_marker_end(data, tag_start + 9, b"]]>", "CDATA section")
            if stack and stack[-1].components_section is not None:
                if not _only_xml_whitespace(data, tag_start + 9, end - 3):
                    raise SourceXmlLayoutError(
                        "non-whitespace CDATA appears directly inside <components>"
                    )
            cursor = end
            continue
        if _starts_with(data, b"<?", tag_start):
            cursor = _find_marker_end(
                data,
                tag_start + 2,
                b"?>",
                "processing instruction",
            )
            continue
        if _starts_with(data, b"<!", tag_start):
            cursor = _find_declaration_end(data, tag_start)
            continue

        tag_end, name, closing, self_closing = _normal_tag(data, tag_start)
        if closing:
            if not stack or stack[-1].name != name:
                expected = stack[-1].name.decode("ascii", "replace") if stack else "none"
                raise SourceXmlLayoutError(
                    "mismatched closing tag {} at byte {}; expected {}".format(
                        name.decode("ascii", "replace"),
                        tag_start,
                        expected,
                    )
                )
            frame = stack.pop()
            if frame.direct_component_start is not None:
                if not stack or stack[-1].components_section is None:
                    raise SourceXmlLayoutError("component parent changed while scanning")
                section = stack[-1].components_section
                section.component_starts.append(frame.direct_component_start)
                section.component_ends.append(tag_end)
            if frame.components_section is not None:
                frame.components_section.content_end = tag_start
            cursor = tag_end
            continue

        is_vehicle_body = (
            name == b"body"
            and len(stack) >= 2
            and stack[-1].name == b"bodies"
            and stack[-2].name == b"vehicle"
        )
        vehicle_body_index: Optional[int] = None
        if is_vehicle_body:
            vehicle_body_index = len(body_sections)
            body_sections.append(None)

        section: Optional[_ComponentsSection] = None
        if (
            name == b"components"
            and stack
            and stack[-1].vehicle_body_index is not None
        ):
            body_index = stack[-1].vehicle_body_index
            if body_sections[body_index] is not None:
                raise SourceXmlLayoutError(
                    "body {} contains multiple direct <components> elements".format(
                        body_index
                    )
                )
            section = _ComponentsSection(
                body_index=body_index,
                content_start=tag_end,
                content_end=tag_end,
                component_starts=array("Q"),
                component_ends=array("Q"),
            )
            body_sections[body_index] = section

        direct_component_start: Optional[int] = None
        if stack and stack[-1].components_section is not None:
            if name != b"c":
                raise SourceXmlLayoutError(
                    "<components> contains direct child <{}>".format(
                        name.decode("ascii", "replace")
                    )
                )
            direct_component_start = tag_start

        frame = _ElementFrame(
            name=name,
            vehicle_body_index=vehicle_body_index,
            components_section=section,
            direct_component_start=direct_component_start,
        )
        if self_closing:
            if direct_component_start is not None:
                parent_section = stack[-1].components_section
                assert parent_section is not None
                parent_section.component_starts.append(direct_component_start)
                parent_section.component_ends.append(tag_end)
            if section is not None:
                section.content_end = tag_end
        else:
            stack.append(frame)
        cursor = tag_end

    if stack:
        raise SourceXmlLayoutError(
            "unterminated <{}> element".format(
                stack[-1].name.decode("ascii", "replace")
            )
        )
    return tuple(body_sections)


def _validate_order(
    body_index: int,
    order: Sequence[int],
    component_count: int,
) -> None:
    if len(order) != component_count:
        raise SourceXmlLayoutError(
            "body {} component order has {} entries; XML has {}".format(
                body_index,
                len(order),
                component_count,
            )
        )
    seen = bytearray(component_count)
    for component_index in order:
        if component_index < 0 or component_index >= component_count:
            raise SourceXmlLayoutError(
                "body {} component order contains out-of-range index {}".format(
                    body_index,
                    component_index,
                )
            )
        if seen[component_index]:
            raise SourceXmlLayoutError(
                "body {} component order repeats index {}".format(
                    body_index,
                    component_index,
                )
            )
        seen[component_index] = 1


def _write_range(
    output: BinaryIO,
    source_view: memoryview,
    start: int,
    end: int,
) -> None:
    if end > start:
        output.write(source_view[start:end])


def write_vehicle_component_order_preserving_source(
    source_path: Path,
    output_path: Path,
    body_component_orders: Sequence[Sequence[int]],
) -> int:
    """Write reordered component chunks while preserving every source byte.

    The output is guaranteed to have exactly the same byte length as the
    source.  Component contents are never parsed or regenerated here; callers
    must still reload the output with the semantic vehicle parser before
    installing it.
    """

    source = Path(source_path)
    destination = Path(output_path)
    if source.resolve() == destination.resolve():
        raise ValueError("source and output paths must be different")
    source_size = os.path.getsize(source)
    if source_size == 0:
        raise SourceXmlLayoutError("vehicle XML is empty")

    with source.open("rb") as source_stream:
        with mmap.mmap(source_stream.fileno(), 0, access=mmap.ACCESS_READ) as mapped:
            sections = _scan_vehicle_components(mapped)
            if len(sections) != len(body_component_orders):
                raise SourceXmlLayoutError(
                    "component orders contain {} bodies; XML has {}".format(
                        len(body_component_orders),
                        len(sections),
                    )
                )

            writable_sections: List[Tuple[_ComponentsSection, Sequence[int]]] = []
            for body_index, (section, order) in enumerate(
                zip(sections, body_component_orders)
            ):
                component_count = (
                    0 if section is None else len(section.component_starts)
                )
                _validate_order(body_index, order, component_count)
                if section is not None:
                    if len(section.component_starts) != len(section.component_ends):
                        raise SourceXmlLayoutError(
                            "body {} component span table is incomplete".format(
                                body_index
                            )
                        )
                    writable_sections.append((section, order))

            destination.parent.mkdir(parents=True, exist_ok=True)
            source_view = memoryview(mapped)
            try:
                with destination.open("wb") as output:
                    cursor = 0
                    for section, order in writable_sections:
                        _write_range(
                            output,
                            source_view,
                            cursor,
                            section.content_start,
                        )
                        component_count = len(section.component_starts)
                        if component_count:
                            _write_range(
                                output,
                                source_view,
                                section.content_start,
                                section.component_starts[0],
                            )
                            for slot_index, component_index in enumerate(order):
                                _write_range(
                                    output,
                                    source_view,
                                    section.component_starts[component_index],
                                    section.component_ends[component_index],
                                )
                                gap_start = section.component_ends[slot_index]
                                gap_end = (
                                    section.component_starts[slot_index + 1]
                                    if slot_index + 1 < component_count
                                    else section.content_end
                                )
                                _write_range(
                                    output,
                                    source_view,
                                    gap_start,
                                    gap_end,
                                )
                        else:
                            _write_range(
                                output,
                                source_view,
                                section.content_start,
                                section.content_end,
                            )
                        cursor = section.content_end
                    _write_range(output, source_view, cursor, len(mapped))
            finally:
                source_view.release()

    output_size = os.path.getsize(destination)
    if output_size != source_size:
        raise RuntimeError(
            "source-preserving XML writer changed byte length: {} -> {}".format(
                source_size,
                output_size,
            )
        )
    return output_size
