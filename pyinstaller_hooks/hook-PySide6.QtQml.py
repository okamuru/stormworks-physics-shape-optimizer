"""Collect only the QML modules used by the embedded Quick 3D viewer.

PyInstaller's stock QtQml hook collects every QML module shipped by the
PySide6-Addons wheel.  That includes unrelated modules such as WebEngine and
adds hundreds of megabytes to this small desktop application.
"""

from pathlib import Path, PurePath

from PyInstaller.utils.hooks.qt import add_qt6_dependencies, pyside6_library_info


hiddenimports, binaries, datas = add_qt6_dependencies(__file__)

qml_source = Path(pyside6_library_info.location["QmlImportsPath"]).resolve()
qml_destination = PurePath(pyside6_library_info.qt_rel_dir) / "qml"

# QtQuick imports the three QtQml modules below.  The application QML directly
# imports QtQuick and QtQuick3D; it does not use Controls, WebEngine, multimedia,
# particles, XR, or the designer-only Quick 3D helper modules.
for relative_module in (
    "QtQml",
    "QtQml/Models",
    "QtQml/WorkerScript",
    "QtQuick",
    "QtQuick3D",
):
    module_path = qml_source / relative_module
    module_binaries, module_datas = pyside6_library_info._process_qml_plugin(
        module_path / "qmldir"
    )
    for source in module_binaries:
        destination = qml_destination / source.relative_to(qml_source).parent
        binaries.append((str(source), str(destination)))
    for source in module_datas:
        relative = source.relative_to(qml_source)
        destination = qml_destination / (relative if source.is_dir() else relative.parent)
        datas.append((str(source), str(destination)))
