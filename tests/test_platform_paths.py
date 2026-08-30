from pathlib import Path
import tempfile
import unittest

from swphysics.platform_paths import (
    config_file,
    definition_directory_candidates,
    find_definition_directory,
    parse_steam_library_paths,
    vehicle_directory,
)


class PlatformPathTests(unittest.TestCase):
    def test_explicit_config_file_does_not_require_replacing_home(self):
        self.assertEqual(
            Path("/tmp/swphysics-test-config.json"),
            config_file(
                "darwin",
                Path("/Users/example"),
                {"SWPHYSICS_CONFIG_FILE": "/tmp/swphysics-test-config.json"},
            ),
        )

    def test_vehicle_directories_for_macos_and_windows(self):
        self.assertEqual(
            Path("/Users/example/Library/Application Support/Stormworks/data/vehicles"),
            vehicle_directory("darwin", Path("/Users/example"), {}),
        )
        self.assertEqual(
            Path("X:/Profile/AppData/Roaming/Stormworks/data/vehicles"),
            vehicle_directory(
                "win32",
                Path("X:/Profile"),
                {"APPDATA": "X:/Profile/AppData/Roaming"},
            ),
        )

    def test_steam_library_vdf_paths_are_parsed(self):
        text = '"0" { "path" "/Volumes/Games/SteamLibrary" }\n"1" { "path" "/Volumes/More" }'
        self.assertEqual(
            [Path("/Volumes/Games/SteamLibrary"), Path("/Volumes/More")],
            parse_steam_library_paths(text, "darwin"),
        )

    def test_custom_macos_library_is_discovered(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            home = Path(temporary_directory)
            steam_root = home / "Library/Application Support/Steam"
            library = home / "ExternalSteam"
            (steam_root / "steamapps").mkdir(parents=True)
            (steam_root / "steamapps/libraryfolders.vdf").write_text(
                '"1" { "path" "' + str(library) + '" }', encoding="utf-8"
            )
            definitions = (
                library
                / "steamapps/common/Stormworks/stormworks.app/Contents/Resources/rom/data/definitions"
            )
            definitions.mkdir(parents=True)
            (definitions / "01_block.xml").write_text("<definition/>", encoding="utf-8")
            self.assertIn(
                definitions, definition_directory_candidates("darwin", home=home, environ={})
            )
            self.assertEqual(
                definitions, find_definition_directory(platform_name="darwin", home=home, environ={})
            )


if __name__ == "__main__":
    unittest.main()
