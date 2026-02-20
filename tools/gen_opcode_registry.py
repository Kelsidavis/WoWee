#!/usr/bin/env python3
"""
Generate opcode registry include fragments from data files.

Inputs:
- Data/opcodes/canonical.json
- Data/opcodes/aliases.json

Outputs:
- include/game/opcode_enum_generated.inc
- include/game/opcode_names_generated.inc
- include/game/opcode_aliases_generated.inc
"""

from __future__ import annotations

import json
import re
from pathlib import Path

RE_NAME = re.compile(r"^(?:CMSG|SMSG|MSG)_[A-Z0-9_]+$")


def load_canonical(path: Path) -> list[str]:
    data = json.loads(path.read_text())
    names = data.get("logical_opcodes", [])
    if not isinstance(names, list):
        raise ValueError("canonical.json: logical_opcodes must be a list")
    out: list[str] = []
    seen: set[str] = set()
    for raw in names:
        if not isinstance(raw, str) or not RE_NAME.match(raw):
            raise ValueError(f"Invalid canonical opcode name: {raw!r}")
        if raw in seen:
            continue
        seen.add(raw)
        out.append(raw)
    return out


def load_aliases(path: Path, canonical: set[str]) -> dict[str, str]:
    data = json.loads(path.read_text())
    aliases = data.get("aliases", {})
    if not isinstance(aliases, dict):
        raise ValueError("aliases.json: aliases must be an object")
    out: dict[str, str] = {}
    for alias, target in sorted(aliases.items()):
        if not isinstance(alias, str) or not RE_NAME.match(alias):
            raise ValueError(f"Invalid alias opcode name: {alias!r}")
        if not isinstance(target, str) or not RE_NAME.match(target):
            raise ValueError(f"Invalid alias target opcode name: {target!r}")
        if target not in canonical:
            raise ValueError(f"Alias target not in canonical set: {alias} -> {target}")
        out[alias] = target
    return out


def write_file(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    data_dir = root / "Data/opcodes"
    inc_dir = root / "include/game"

    canonical_names = load_canonical(data_dir / "canonical.json")
    canonical_set = set(canonical_names)
    aliases = load_aliases(data_dir / "aliases.json", canonical_set)

    enum_lines = ["// GENERATED FILE - DO NOT EDIT", ""]
    enum_lines += [f"    {name}," for name in canonical_names]
    enum_content = "\n".join(enum_lines) + "\n"

    name_lines = ["// GENERATED FILE - DO NOT EDIT", ""]
    name_lines += [f'    {{"{name}", LogicalOpcode::{name}}},' for name in canonical_names]
    names_content = "\n".join(name_lines) + "\n"

    alias_lines = ["// GENERATED FILE - DO NOT EDIT", ""]
    alias_lines += [f'    {{"{alias}", "{target}"}},' for alias, target in aliases.items()]
    aliases_content = "\n".join(alias_lines) + "\n"

    write_file(inc_dir / "opcode_enum_generated.inc", enum_content)
    write_file(inc_dir / "opcode_names_generated.inc", names_content)
    write_file(inc_dir / "opcode_aliases_generated.inc", aliases_content)

    print(
        f"generated: canonical={len(canonical_names)} aliases={len(aliases)} "
        f"-> include/game/opcode_*_generated.inc"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
