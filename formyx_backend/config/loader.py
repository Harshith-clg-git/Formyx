"""
formyx_backend/config/loader.py
--------------------------------
Centralised configuration loader.  All modules import from here so
the YAML file is only parsed once per process.
"""

from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Any

import yaml

_CONFIG_PATH = Path(__file__).parent / "settings.yaml"
_lock = threading.Lock()
_config: dict[str, Any] | None = None


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """
    Load (and cache) the global settings YAML.

    Parameters
    ----------
    path:
        Override the default config file path.  Useful in tests.

    Returns
    -------
    dict
        Parsed configuration dictionary.
    """
    global _config

    with _lock:
        if _config is not None and path is None:
            return _config

        config_path = Path(path) if path else _CONFIG_PATH
        if not config_path.exists():
            raise FileNotFoundError(
                f"Configuration file not found: {config_path}\n"
                "Expected location: formyx_backend/config/settings.yaml"
            )

        with config_path.open("r", encoding="utf-8") as fh:
            parsed = yaml.safe_load(fh)

        if path is None:
            _config = parsed  # cache only the default config

        return parsed


def get(section: str, key: str, default: Any = None) -> Any:
    """
    Convenience helper — fetch a nested config value.

    Example
    -------
    >>> get("mavlink", "connection_string")
    'udpin:localhost:14550'
    """
    cfg = load_config()
    return cfg.get(section, {}).get(key, default)
