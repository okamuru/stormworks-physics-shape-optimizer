from pathlib import Path
import tempfile
import unittest

from swphysics.source_preserving_xml import (
    SourceXmlLayoutError,
    write_vehicle_component_order_preserving_source,
)


class SourcePreservingXmlTests(unittest.TestCase):
    def _write(self, source_bytes, orders):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        source = root / "source.xml"
        output = root / "output.xml"
        source.write_bytes(source_bytes)
        written = write_vehicle_component_order_preserving_source(
            source,
            output,
            orders,
        )
        return source, output, written

    def test_identity_order_is_byte_identical_with_bom_crlf_and_doctype(self):
        source_bytes = (
            b"\xef\xbb\xbf<?xml version='1.0' encoding='UTF-8'?>\r\n"
            b"<!DOCTYPE vehicle [<!ENTITY marker 'kept'>]>\r\n"
            b"<vehicle><bodies><body><components>\r\n"
            b"  <c d='first' note='x > y'><o><![CDATA[fake <c/> text]]></o></c>\r\n"
            b"  <!-- fixed slot comment -->\r\n"
            b"  <c d=\"second\" 00=\"1\"><object><c d='nested'/></object></c>\r\n"
            b"</components></body></bodies></vehicle>"
        )
        _source, output, written = self._write(source_bytes, ((0, 1),))

        self.assertEqual(len(source_bytes), written)
        self.assertEqual(source_bytes, output.read_bytes())

    def test_reorder_moves_only_direct_component_chunks_and_keeps_size(self):
        source_bytes = (
            b"<vehicle><bodies><body><components>\n"
            b" <c id='first'><object><c id='nested'/></object></c>\n"
            b" <?slot keep?>\n"
            b" <c id=\"second\" note=\"1 > 0\"/>\n"
            b"</components></body>"
            b"<body><components><c id='third'/></components></body>"
            b"</bodies></vehicle>\n"
        )
        _source, output, written = self._write(
            source_bytes,
            ((1, 0), (0,)),
        )
        actual = output.read_bytes()

        self.assertEqual(len(source_bytes), written)
        self.assertEqual(len(source_bytes), len(actual))
        self.assertIn(
            b"<components>\n <c id=\"second\" note=\"1 > 0\"/>\n"
            b" <?slot keep?>\n"
            b" <c id='first'><object><c id='nested'/></object></c>\n"
            b"</components>",
            actual,
        )
        self.assertTrue(actual.endswith(b"</bodies></vehicle>\n"))

    def test_vehicle_authors_are_preserved_and_never_replaced_by_app_author(self):
        source_bytes = (
            b"<vehicle><authors><author name='Original Vehicle Creator'/>"
            b"</authors><bodies><body><components>"
            b"<c id='first'/><c id='second'/>"
            b"</components></body></bodies></vehicle>"
        )
        _source, output, written = self._write(source_bytes, ((1, 0),))
        actual = output.read_bytes()

        self.assertEqual(len(source_bytes), written)
        self.assertIn(
            b"<authors><author name='Original Vehicle Creator'/></authors>",
            actual,
        )
        self.assertNotIn(b"IrisNuiYaMa_164", actual)

    def test_empty_and_self_closing_component_collections_are_supported(self):
        source_bytes = (
            b"<vehicle><bodies><body/><body><components/></body>"
            b"<body><components></components></body></bodies></vehicle>"
        )
        _source, output, _written = self._write(
            source_bytes,
            ((), (), ()),
        )
        self.assertEqual(source_bytes, output.read_bytes())

    def test_invalid_order_fails_closed(self):
        source_bytes = (
            b"<vehicle><bodies><body><components>"
            b"<c/><c/>"
            b"</components></body></bodies></vehicle>"
        )
        for order, message in (
            ((0,), "entries"),
            ((0, 0), "repeats"),
            ((0, 2), "out-of-range"),
        ):
            with self.subTest(order=order):
                with self.assertRaisesRegex(SourceXmlLayoutError, message):
                    self._write(source_bytes, (order,))

    def test_non_whitespace_text_in_components_fails_closed(self):
        source_bytes = (
            b"<vehicle><bodies><body><components>unsafe<c/>"
            b"</components></body></bodies></vehicle>"
        )
        with self.assertRaisesRegex(SourceXmlLayoutError, "non-whitespace text"):
            self._write(source_bytes, ((0,),))


if __name__ == "__main__":
    unittest.main()
