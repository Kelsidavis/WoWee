#!/usr/bin/env python3
"""Settings in the schema that never reach the config file.

    tools/settings_persist_check.py

WHY

The settings screens are generated from one list - kSchema in
settings_schema.cpp - and every entry there becomes a control in both the
client's own window and the interface's options panels. Saving is not
generated: GameScreen::saveSettings writes each field out by hand, and
loadSettings reads them back the same way.

So a setting can be added to the schema, appear in both windows, answer its
value, apply correctly, and be gone at the next login. Nothing raises. The
control is there, the value works, and it resets every time the client starts -
which reads to a player as "that option doesn't stick" and to a developer as
nothing at all, because every test still passes.

Forty-nine settings were added to that schema in one session. This is the check
that says whether they will survive a relog.

WHAT IT DOES

For every schema key, finds the field it binds to in settings_panel.cpp's
binding tables, then asks whether that field is both written by saveSettings and
read by loadSettings. A field missing from either is reported.

WHAT IT CANNOT SEE

Whether the value written is the value meant, or whether the key names in the
file are stable across versions. It answers "is there a writer and a reader",
which is the question that was silently false.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCHEMA = ROOT / "src/ui/settings_schema.cpp"
PANEL = ROOT / "src/ui/settings_panel.cpp"
PERSIST = ROOT / "src/ui/game_screen_minimap.cpp"

# Settings with no stored field of their own, and why each.
EXPECTED = {
    # Not stored: it is whatever the settings it covers currently amount to,
    # worked out by updateGraphicsPresetFromCurrentSettings on load.
    "graphicspreset": "derived from the settings it covers",
}


def schema_keys(text):
    return re.findall(r'\{(?:\.\w+\s*=\s*)?"([a-z0-9]+)",\s*(?:\.\w+\s*=\s*)?"[^"]*",\s*(?:\.\w+\s*=\s*)?SettingKind::', text)


def bindings(text):
    """schema key -> the member it reads and writes."""
    out = {}
    for m in re.finditer(
            r'\.key\s*=\s*"([a-z0-9]+)"\s*,\s*\.as\w+\s*=\s*&SettingsPanel::(\w+)', text, re.S):
        out[m.group(1)] = m.group(2)
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([a-z0-9]+)",\s*(?:\.\w+\s*=\s*)?&ChatSettings::(\w+)\}', text):
        # The chat panel exposes these under chatAutoJoin* aliases.
        out[m.group(1)] = m.group(2)
    return out


def body(text, signature):
    """A function's text, from its signature to the next one at column zero."""
    start = text.index(signature)
    rest = text[start + len(signature):]
    end = re.search(r"\n\w[\w:&<>* ]*\w::\w+\(", rest)
    return rest[: end.start()] if end else rest


CHAT_PANEL = ROOT / "include/ui/chat_panel.hpp"


def chatAliases():
    """ChatSettings member -> the reference ChatPanel names it by.

    The saver and loader write the reference, not the member, so a binding is
    reached under either name. Read from the header rather than spelled here:
    the rule used to be "autoJoin becomes chatAutoJoin", which was every alias
    there was until the speech-bubble rows arrived under names of their own.
    """
    out = {}
    for m in re.finditer(r'\b\w+&\s+(\w+)\s*=\s*settings\.(\w+)\s*;', CHAT_PANEL.read_text()):
        out[m.group(2)] = m.group(1)
    return out


def mentions(where, member):
    return member in where or chatAliases().get(member, member) in where


def main():
    schema = SCHEMA.read_text()
    panel = PANEL.read_text()
    persist = PERSIST.read_text()

    keys = schema_keys(schema)
    if not keys:
        print("Could not read the schema. Every count below is meaningless.")
        return 1
    binds = bindings(panel)

    save = body(persist, "void GameScreen::saveSettings")
    load = body(persist, "void GameScreen::loadSettings")
    if not save or not load:
        print("Could not find saveSettings or loadSettings. Do not believe the zero.")
        return 1

    unbound, unsaved, unloaded = [], [], []
    for key in keys:
        if key in EXPECTED:
            continue
        member = binds.get(key)
        if not member:
            unbound.append(key)
            continue
        if not mentions(save, member):
            unsaved.append((key, member))
        if not mentions(load, member):
            unloaded.append((key, member))

    print(f"{len(keys)} settings in the schema, {len(EXPECTED)} settled\n")

    rows = [("no field behind it", unbound and [(k, "") for k in unbound] or []),
            ("never written to the config file", unsaved),
            ("written but never read back", unloaded)]
    total = 0
    for title, items in rows:
        print(f"{len(items)} {title}:")
        for k, m in items:
            print(f"  {k:22} {m}")
        if not items:
            print("  (none)")
        print()
        total += len(items)
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
