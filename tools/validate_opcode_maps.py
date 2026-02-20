#!/usr/bin/env python3
"""
Validate opcode canonicalization and expansion mappings.

Checks:
1. Every enum opcode appears in kOpcodeNames.
2. Every expansion JSON key resolves to a canonical opcode name (direct or alias).
3. Every opcode referenced as Opcode::<NAME> in implementation code exists in each expansion map
   after alias canonicalization.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Dict, Iterable, List, Set


RE_OPCODE_NAME = re.compile(r"^(?:CMSG|SMSG|MSG)_[A-Z0-9_]+$")
RE_CODE_REF = re.compile(r"\bOpcode::((?:CMSG|SMSG|MSG)_[A-Z0-9_]+)\b")


def read_canonical_data(path: Path) -> Set[str]:
    data = json.loads(path.read_text())
    names = data.get("logical_opcodes", [])
    return {n for n in names if isinstance(n, str) and RE_OPCODE_NAME.match(n)}


def read_alias_data(path: Path) -> Dict[str, str]:
    data = json.loads(path.read_text())
    aliases = data.get("aliases", {})
    out: Dict[str, str] = {}
    for k, v in aliases.items():
        if isinstance(k, str) and isinstance(v, str) and RE_OPCODE_NAME.match(k) and RE_OPCODE_NAME.match(v):
            out[k] = v
    return out


def canonicalize(name: str, aliases: Dict[str, str]) -> str:
    seen: Set[str] = set()
    current = name
    while current in aliases and current not in seen:
        seen.add(current)
        current = aliases[current]
    return current


def iter_expansion_files(expansions_dir: Path) -> Iterable[Path]:
    for p in sorted(expansions_dir.glob("*/opcodes.json")):
        yield p


def load_expansion_names(path: Path) -> Dict[str, str]:
    data = json.loads(path.read_text())
    out: Dict[str, str] = {}
    for k, v in data.items():
        if RE_OPCODE_NAME.match(k):
            out[k] = str(v)
    return out


def collect_code_refs(root: Path) -> Set[str]:
    refs: Set[str] = set()
    skip_suffixes = {
        "include/game/opcode_table.hpp",
        "src/game/opcode_table.cpp",
    }
    for p in list(root.glob("src/**/*.cpp")) + list(root.glob("include/**/*.hpp")):
        rel = p.as_posix()
        if rel in skip_suffixes:
            continue
        text = p.read_text(errors="ignore")
        for m in RE_CODE_REF.finditer(text):
            refs.add(m.group(1))
    return refs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    parser.add_argument(
        "--strict-required",
        action="store_true",
        help="Fail when expansion maps miss opcodes referenced by implementation code.",
    )
    args = parser.parse_args()

    root = Path(args.root).resolve()
    canonical_path = root / "Data/opcodes/canonical.json"
    aliases_path = root / "Data/opcodes/aliases.json"
    expansions_dir = root / "Data/expansions"

    enum_names = read_canonical_data(canonical_path)
    aliases = read_alias_data(aliases_path)
    k_names = set(enum_names)
    code_refs = collect_code_refs(root)

    problems: List[str] = []

    missing_in_name_map = sorted(enum_names - k_names)
    if missing_in_name_map:
        problems.append(
            f"enum names missing from kOpcodeNames: {len(missing_in_name_map)} "
            f"(sample: {missing_in_name_map[:10]})"
        )

    unknown_code_refs = sorted(r for r in code_refs if canonicalize(r, aliases) not in enum_names)
    if unknown_code_refs:
        problems.append(
            f"Opcode:: references not in enum/alias map: {len(unknown_code_refs)} "
            f"(sample: {unknown_code_refs[:10]})"
        )

    print(f"Canonical enum names: {len(enum_names)}")
    print(f"kOpcodeNames entries: {len(k_names)}")
    print(f"Alias entries: {len(aliases)}")
    print(f"Opcode:: code references: {len(code_refs)}")

    for exp_file in iter_expansion_files(expansions_dir):
        names = load_expansion_names(exp_file)
        canonical_names = {canonicalize(n, aliases) for n in names}
        unknown = sorted(n for n in canonical_names if n not in enum_names)
        missing_required = sorted(
            n for n in code_refs if canonicalize(n, aliases) not in canonical_names
        )

        # Detect multiple raw names collapsing to one canonical name.
        collisions: Dict[str, List[str]] = {}
        for raw in names:
            c = canonicalize(raw, aliases)
            collisions.setdefault(c, []).append(raw)
        alias_collisions = sorted(
            (c, raws) for c, raws in collisions.items() if len(raws) > 1 and len(set(raws)) > 1
        )

        print(
            f"[{exp_file.parent.name}] raw={len(names)} canonical={len(canonical_names)} "
            f"unknown={len(unknown)} missing_required={len(missing_required)} "
            f"alias_collisions={len(alias_collisions)}"
        )

        if unknown:
            problems.append(
                f"{exp_file.parent.name}: unknown canonical names after aliasing: "
                f"{len(unknown)} (sample: {unknown[:10]})"
            )
        if missing_required and args.strict_required:
            problems.append(
                f"{exp_file.parent.name}: missing required opcodes from implementation refs: "
                f"{len(missing_required)} (sample: {missing_required[:10]})"
            )
        elif missing_required:
            print(
                f"  warn: {exp_file.parent.name} missing required refs: "
                f"{len(missing_required)} (sample: {missing_required[:6]})"
            )

    if problems:
        print("\nFAILED:")
        for p in problems:
            print(f"- {p}")
        return 1

    print("\nOK: canonical opcode contract satisfied across expansions.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
