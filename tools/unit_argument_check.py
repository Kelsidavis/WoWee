#!/usr/bin/env python3
"""Unit bindings that never look at the unit they were asked about.

    tools/unit_argument_check.py

A binding whose name starts with Unit takes a unit token as its first argument -
"player", "target", "pet", "party1" - and every one of them is asked about more
than one unit by something in FrameXML. A binding that ignores that argument and
answers from the player does not fail: it returns a real number belonging to the
wrong character, which is the hardest kind of wrong to see. Nothing is empty,
nothing is zero, nothing raises.

This turned up three times in one day, in three files:

  * SetInventoryItem, which showed the player's own equipment on the inspect
    paperdoll - recorded in its own comment as a bug fixed once already.
  * UnitStat and UnitResistance, which listed a hunter's Strength and armour as
    the pet's on the paperdoll's pet tab.
  * UnitArmor, the same, through PaperDollFrame_SetArmor - which the pet tab
    calls with "Pet", capitalised, so a comparison against "pet" has to lower
    the token first.

WHAT IT LOOKS FOR

A binding with Unit in its name that reads no first argument, calls no unit
resolver, and answers out of player-only state. All three conditions, because
plenty of Unit bindings legitimately answer for the player alone - UnitXP and
UnitCharacterPoints are asked about nothing else by anything here.

WHAT IT CANNOT SEE

A binding that resolves its unit and then uses the answer wrongly. It also
cannot tell a binding that *should* answer for one unit only from one that has
simply not been asked yet; that judgement belongs in the commit that leaves it.

Two shapes report as faults and are not:

  * the unit is not the first argument - IsUnitOnQuest takes a quest index
    first, so nothing is read at position one;
  * the binding hands L straight to another that does resolve -
    UnitPlayerOrPetInRaid delegates to UnitPlayerOrPetInParty.

Both are cheap to recognise by eye and expensive to encode, so they are left in
the count and named here.

THE TEN IT REPORTS TODAY, EACH OPENED (2026-08-05)

  * Five paperdoll figures - UnitDefense, UnitAttackBothHands, UnitRangedAttack,
    UnitRangedAttackPower, UnitRangedDamage. FrameXML's PaperDollFrame_Set*
    helpers take (statFrame, unit) and fall back to "player" when the unit is
    absent, and every caller of these five passes a frame and no unit. The pet
    sheet passes a unit to exactly three helpers - SetDamage, SetArmor and
    SetAttackPower - and none of them is one of these five. So they are asked
    about the player alone, and the day the pet sheet grows a defence or ranged
    row is the day this stops being true.
  * UnitIsSameServer - answers true always, which is what "same realm" means
    where there is no cross-realm anything.
  * UnitControllingVehicle - asked about "player" and nothing else.
  * GetUnitHealthModifier - a constant 1.0, and the one row here whose value a
    real per-unit answer would change: petpaperdollframe multiplies the stamina
    tooltip's projected health gain by it, for "pet".
  * IsUnitOnQuest and UnitPlayerOrPetInRaid - the two shapes above.

THE TRAP THE THREE PET-SHEET HELPERS SET

They pass the token **capitalised** - PaperDollFrame_SetArmor(PetArmorFrame,
"Pet") - so any binding they reach has to lower it before comparing. UnitDamage,
UnitArmor and UnitAttackPower all do, and all three read the pet's own fields;
that is what makes them absent from this list rather than on it.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ADDONS = ROOT / "src/addons"

#: Reading the first argument, or resolving it, counts as looking at the unit.
LOOKS = re.compile(r"resolveUnit|resolveUnitGuid"
                   r"|luaL_(?:opt|check)string\(\s*L\s*,\s*1")
#: State that belongs to the player and nobody else.
#:
#: An accessor taking no argument cannot be answering about an arbitrary unit -
#: there is nowhere to say which one - so `gh->getSomething()` with empty
#: parentheses is the general form of the fault. Naming individual getters
#: instead is what let UnitAttackPower through: it reads getMeleeAttackPower(),
#: which no list of names written in advance happened to contain.
PLAYER_ONLY = re.compile(r"\bgh->get\w+\(\s*\)|playerGuid")


def bindings(text):
    """(name, body) for both the named and the inline binding shapes.

    The inline body is found by matching braces rather than by looking for the
    closing "}}" at a fixed indent. That indent only exists on a lambda written
    across several lines, so every one-line lambda was invisible here - 162 of
    them, four with a unit in the name - and worse, a multi-line lambda's body
    ran on to the next "}}" it could find and swallowed whatever was between.
    Which bindings got reported therefore depended on how the ones above them
    happened to be formatted: reformatting one of them changed the count.
    """
    # Braces matched here too. A named function written on one line has no
    # closing brace at the start of a line, so the non-greedy form ran on and
    # inherited the next function's body - the same fault the inline half had.
    for m in re.finditer(r"static int (lua_\w+)\(lua_State\* L\)\s*\{(?:\.\w+\s*=\s*)?", text):
        depth, i = 1, m.end()
        while i < len(text) and depth:
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
            i += 1
        yield m.group(1), text[m.end():i - 1]
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"(\w+)",\s*(?:\.\w+\s*=\s*)?\[\]\(lua_State\* L\) -> int \{', text):
        depth, i = 1, m.end()
        while i < len(text) and depth:
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
            i += 1
        yield m.group(1), text[m.end():i - 1]


#: Bindings whose answer is the player's on purpose, with what settled each.
#:
#: A set rather than a count: a ceiling only says how many there are, so fixing
#: one and introducing another leaves the number unmoved. handler_announce_check
#: was pinned that way and hid a live bug for as long as it was.
#:
#: The five paperdoll stats are read through PaperDollFrame_Set… helpers whose
#: every call site passes no unit at all, and each opens by defaulting it to
#: "player". There is no path that asks them about anyone else.
EXPECTED = {
    "lua_UnitAttackBothHands": "paperdoll stat, every caller defaults to player",
    "lua_UnitDefense": "paperdoll stat, every caller defaults to player",
    "lua_UnitRangedAttack": "paperdoll stat, every caller defaults to player",
    "lua_UnitRangedAttackPower": "paperdoll stat, every caller defaults to player",
    "lua_UnitRangedDamage": "paperdoll stat, every caller defaults to player",
    # The server never tells a client what another player's quest log holds, so
    # only the player is answerable. False for everyone else leaves the shared
    # count hidden rather than claiming a number nobody can stand behind.
    "IsUnitOnQuest": "no data for any unit but the player",
}

#: `return lua_Other(L);` - the binding hands the whole call, unit argument and
#: all, to another one. UnitPlayerOrPetInRaid does exactly that to
#: UnitPlayerOrPetInParty, which resolves the unit properly, and reading only
#: the first body reported it as answering from the player.
DELEGATES = re.compile(r"return\s+(lua_\w+)\s*\(\s*L\s*\)")


def main():
    # Every binding body, so a delegation can be followed to what it calls.
    all_bodies = {}
    for path in sorted(ADDONS.glob("*.cpp")):
        for name, body in bindings(path.read_text(errors="ignore")):
            all_bodies.setdefault(name, body)

    def looks_at_unit(name, body, depth=0):
        if LOOKS.search(body):
            return True
        if depth >= 3:
            return False
        for callee in DELEGATES.findall(body):
            target = all_bodies.get(callee)
            if target is not None and looks_at_unit(callee, target, depth + 1):
                return True
        return False

    rows = []
    examined = 0
    for path in sorted(ADDONS.glob("*.cpp")):
        text = path.read_text(errors="ignore")
        for name, body in bindings(text):
            examined += 1
            if "Unit" not in name:
                continue
            if name in EXPECTED:
                continue
            if looks_at_unit(name, body):
                continue
            if not PLAYER_ONLY.search(body):
                continue
            rows.append((name, path.name))

    rows = sorted(set(rows))
    # What it looked at, before what it found. A sweep pinned at zero that
    # reports only its findings reads the same whether the tree is clean or its
    # parser has stopped recognising a binding.
    if not examined:
        print("Found no unit bindings at all, which cannot be right - the "
              "binding parser broke rather than every unit binding being "
              "removed.")
        return 1
    print(f"{examined} unit binding(s) examined\n")
    print(f"{len(rows)} unit binding(s) that never look at their unit and "
          f"answer from the player:\n")
    for name, where in rows:
        print(f"  {name:36} {where}")
    if not rows:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
