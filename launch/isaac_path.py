"""Helpers to make Isaac Sim Python modules discoverable.

This module is intentionally free of Isaac Sim imports so it can run in
environments where Isaac Sim is not installed and emit a helpful error
message.
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import sys
from typing import Iterable, List


def _collect_candidate_paths() -> List[str]:
    """Return potential paths that may contain the Isaac Sim Python packages."""

    candidates: List[str] = []

    env_paths = os.environ.get("ISAACSIM_PYTHON_PATH")
    if env_paths:
        candidates.extend(env_paths.split(os.pathsep))

    root = os.environ.get("ISAACSIM_ROOT") or os.environ.get("ISAACSIM_PATH")
    if root:
        root_path = pathlib.Path(root)
        for suffix in ("python", "kit/python"):
            candidate = root_path / suffix
            candidates.append(str(candidate))

    if not candidates:
        candidates.extend(_discover_default_python_paths())

    return candidates


def _prepend_paths(paths: Iterable[str]) -> None:
    for path in paths:
        if path and path not in sys.path:
            sys.path.insert(0, path)


def _discover_default_python_paths() -> List[str]:
    """Return Python import paths from common Isaac Sim 5.x install locations.

    Isaac Sim 5.x installs default to ``~/.local/share/ov/pkg`` on Linux and
    ``%LOCALAPPDATA%/ov/pkg`` on Windows. We probe these locations for
    ``isaac-sim-5.*`` directories and add their Python subfolders so users can
    run the launchers without manually setting environment variables.
    """

    paths: List[str] = []

    def _candidate_roots(base_dir: pathlib.Path) -> List[pathlib.Path]:
        if not base_dir.is_dir():
            return []
        return sorted(
            (d for d in base_dir.iterdir() if d.is_dir() and d.name.startswith("isaac-sim-5")),
            reverse=True,
        )

    linux_base = pathlib.Path.home() / ".local" / "share" / "ov" / "pkg"
    windows_base = pathlib.Path(
        os.environ.get("LOCALAPPDATA", pathlib.Path.home() / "AppData" / "Local")
    ) / "ov" / "pkg"

    for base in (linux_base, windows_base):
        for root in _candidate_roots(base):
            for suffix in ("python", "kit/python"):
                paths.append(str(root / suffix))

    return paths


def ensure_isaac_python_path() -> None:
    """Populate ``sys.path`` with Isaac Sim locations or raise a clear error.

    Users should run the launchers with the Isaac Sim Python interpreter
    (``python.sh`` on Linux/macOS or ``python.bat`` on Windows). This helper
    adds paths from the conventional environment variables so scripts remain
    usable when executed directly, while still failing fast with actionable
    guidance if Isaac Sim is unavailable.
    """

    _prepend_paths(_collect_candidate_paths())

    if importlib.util.find_spec("omni.isaac.core") is None:
        raise ModuleNotFoundError(
            "omni.isaac.core not found. Launch using the Isaac Sim Python "
            "interpreter (python.sh/python.bat) or set ISAACSIM_PYTHON_PATH to "
            "<isaac-sim-root>/python."
        )

