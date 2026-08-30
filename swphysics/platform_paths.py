import json
import os
from pathlib import Path, PureWindowsPath
import re
import sys
from typing import Dict, Iterable, List, Mapping, Optional, Sequence


APP_DIRECTORY_NAME = "StormworksPhysicsShapeOptimizer"


def _platform_name(platform_name: Optional[str] = None) -> str:
    return platform_name or sys.platform


def vehicle_directory(
    platform_name: Optional[str] = None,
    home: Optional[Path] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> Path:
    platform_value = _platform_name(platform_name)
    home_path = Path(home) if home is not None else Path.home()
    environment = dict(os.environ if environ is None else environ)
    if platform_value == "darwin":
        return home_path / "Library/Application Support/Stormworks/data/vehicles"
    if platform_value.startswith("win"):
        appdata = environment.get("APPDATA")
        if appdata:
            return Path(appdata) / "Stormworks/data/vehicles"
        return home_path / "AppData/Roaming/Stormworks/data/vehicles"
    return home_path / ".local/share/Stormworks/data/vehicles"


def config_file(
    platform_name: Optional[str] = None,
    home: Optional[Path] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> Path:
    platform_value = _platform_name(platform_name)
    home_path = Path(home) if home is not None else Path.home()
    environment = dict(os.environ if environ is None else environ)
    explicit = environment.get("SWPHYSICS_CONFIG_FILE")
    if explicit:
        return Path(explicit)
    if platform_value == "darwin":
        return home_path / "Library/Application Support" / APP_DIRECTORY_NAME / "config.json"
    if platform_value.startswith("win"):
        appdata = environment.get("APPDATA")
        base = Path(appdata) if appdata else home_path / "AppData/Roaming"
        return base / APP_DIRECTORY_NAME / "config.json"
    return home_path / ".config" / APP_DIRECTORY_NAME / "config.json"


def parse_steam_library_paths(text: str, platform_name: Optional[str] = None) -> List[Path]:
    platform_value = _platform_name(platform_name)
    results: List[Path] = []
    for raw_path in re.findall(r'"path"\s+"([^"]+)"', text):
        decoded = raw_path.replace("\\\\", "\\")
        path = Path(PureWindowsPath(decoded)) if platform_value.startswith("win") else Path(decoded)
        if path not in results:
            results.append(path)
    return results


def _windows_steam_registry_paths() -> List[Path]:
    if not sys.platform.startswith("win"):
        return []
    try:
        import winreg  # type: ignore
    except ImportError:
        return []
    results = []
    registry_locations = (
        (winreg.HKEY_CURRENT_USER, r"Software\Valve\Steam", "SteamPath"),
        (winreg.HKEY_LOCAL_MACHINE, r"Software\WOW6432Node\Valve\Steam", "InstallPath"),
    )
    for hive, key_name, value_name in registry_locations:
        try:
            with winreg.OpenKey(hive, key_name) as key:
                value, _kind = winreg.QueryValueEx(key, value_name)
            candidate = Path(value)
            if candidate not in results:
                results.append(candidate)
        except OSError:
            continue
    return results


def steam_root_candidates(
    platform_name: Optional[str] = None,
    home: Optional[Path] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> List[Path]:
    platform_value = _platform_name(platform_name)
    home_path = Path(home) if home is not None else Path.home()
    environment = dict(os.environ if environ is None else environ)
    candidates: List[Path] = []
    if platform_value == "darwin":
        candidates.append(home_path / "Library/Application Support/Steam")
    elif platform_value.startswith("win"):
        candidates.extend(_windows_steam_registry_paths())
        for key in ("PROGRAMFILES(X86)", "PROGRAMFILES"):
            value = environment.get(key)
            if value:
                candidates.append(Path(value) / "Steam")
        candidates.append(Path(r"C:\Program Files (x86)\Steam"))
    else:
        candidates.extend((home_path / ".steam/steam", home_path / ".local/share/Steam"))
    unique = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    return unique


def _definition_path_for_library(library: Path, platform_name: str) -> Path:
    game_root = library / "steamapps/common/Stormworks"
    if platform_name == "darwin":
        return game_root / "stormworks.app/Contents/Resources/rom/data/definitions"
    return game_root / "rom/data/definitions"


def definition_directory_candidates(
    platform_name: Optional[str] = None,
    home: Optional[Path] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> List[Path]:
    platform_value = _platform_name(platform_name)
    results: List[Path] = []
    for steam_root in steam_root_candidates(platform_value, home, environ):
        libraries = [steam_root]
        vdf_path = steam_root / "steamapps/libraryfolders.vdf"
        try:
            libraries.extend(parse_steam_library_paths(vdf_path.read_text(encoding="utf-8"), platform_value))
        except (OSError, UnicodeError):
            pass
        for library in libraries:
            candidate = _definition_path_for_library(library, platform_value)
            if candidate not in results:
                results.append(candidate)
    return results


def find_definition_directory(
    preferred: Optional[Path] = None,
    platform_name: Optional[str] = None,
    home: Optional[Path] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> Optional[Path]:
    candidates: List[Path] = []
    if preferred is not None:
        candidates.append(Path(preferred))
    candidates.extend(definition_directory_candidates(platform_name, home, environ))
    for candidate in candidates:
        if (candidate / "01_block.xml").is_file():
            return candidate
    return None


def load_config(path: Optional[Path] = None) -> Dict[str, str]:
    target = Path(path) if path is not None else config_file()
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return {str(key): str(value) for key, value in data.items() if isinstance(value, str)}


def save_config(values: Mapping[str, str], path: Optional[Path] = None) -> None:
    target = Path(path) if path is not None else config_file()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(
        json.dumps(dict(values), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    os.replace(str(temporary), str(target))
