#!/usr/bin/env python3
"""Packet fields the parser fills in and nothing ever reads.

    tools/parsed_never_read_check.py

WHY THIS FINDS WHAT THE OTHER SWEEPS CANNOT

Every readiness and stub check works down from the interface: which name does
FrameXML call, and does it answer. This works up from the wire instead. The
server sent a value, the parser stored it, the struct carries it - and no line
outside the parser mentions it. A field nobody reads is usually a branch nobody
runs, and it is invisible from the interface side because the binding that
should have consulted it looks complete: it returns a number, just always the
same one.

Two found the day this was written, both live:

  * LootItem.lootSlotType. GetLootSlotInfo answered `locked` false with the
    note "not tracked", while the slot type sat parsed beside it. Rolls still
    running and master-loot items drew as ordinary loot and did nothing when
    clicked.
  * GroupInviteResponseData.canAccept. AzerothCore sends SMSG_GROUP_INVITE with
    a zero in that byte to mean "already in a group, this could not be
    offered". Ignored, so the refusal raised the accept/decline popup and left
    an invite pending that would never be honoured.

Both had a comment nearby asserting the data was not available. That is the
shape: the field is the evidence against the comment, and only a sweep from the
wire side puts the two next to each other.

WHAT IT COMPARES

Fields of the structs in include/game/world_packets.hpp, against every mention
of that name in the tree. A field counts as read when it is named anywhere
outside the packet header and the parser sources.

TWO THINGS THAT MAKE A NAIVE VERSION REPORT ZERO FOREVER

Both were hit while writing this, and both fail silently in the safe-looking
direction - a clean report that has seen nothing.

  * The declaring header counts as a mention. Reading it as a use makes every
    field look read, and the sweep reports zero for all time. The canary below
    exists because that is exactly what the first run did.
  * A field read only through an inline accessor on its own struct is not
    named anywhere else. QuestRequestItemsData.completableFlags is read all
    over the client through isCompletable(), and would be a false finding.
    Accessors defined in the struct are followed, and a field reached through
    one that somebody calls is reported separately rather than as a fault.

WHAT IT CANNOT SEE

A field read through a name built at runtime, a memcpy over the whole struct,
or a field whose *value* is wrong rather than unread. And it says nothing about
whether the reader does the right thing with what it finds - only that there is
one.
"""
import collections
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
HDR = ROOT / "include/game/world_packets.hpp"
PARSER = re.compile(r"world_packets.*\.cpp$")

#: Fields checked once and found to be correctly unread, with the reason. A set
#: rather than a count: a ceiling of twelve cannot tell a field being answered
#: from a new one arriving to replace it.
EXPECTED = {
    # 3.3.5's GetLootSlotInfo returns five values and none of them is this one;
    # the quest border on a loot square arrives in a later expansion. The bag
    # and bank frames do draw one, from GetContainerItemQuestInfo, which is a
    # different source entirely.
    ("LootItem", "isQuestItem"): "no consumer in 3.3.5's loot frame",
    # Named unknown because it is. Reading it would be inventing a meaning.
    ("AuthChallengeData", "unknown1"): "unidentified field, by name",
    # Echoed, not decided: AzerothCore reads emoteNum off CMSG_TEXT_EMOTE and
    # relays it untouched, so every observer shows the phrasing the *sender*
    # picked. It selects among an emote's several wordings - and this client's
    # EmoteRegistry keeps exactly one othersTarget and one othersNoTarget per
    # emote, so there is no second wording to select. Sending zero and ignoring
    # what comes back agree with each other. It becomes a real finding the day
    # the registry carries variants.
    ("TextEmoteData", "emoteNum"): "no text variants to choose between",
    # Read correctly now - the condition was inverted and the field was taken
    # exactly when it was absent - but there is nothing behind it on this
    # server. AzerothCore's only four-argument SendAuctionCommandResult passes
    # a literal zero and the parameter defaults to zero everywhere else, so a
    # reader would have nothing but zero to report. In the protocol at large it
    # says a bid was outbid the moment it was placed.
    ("AuctionCommandResult", "bidError"): "always zero as AzerothCore sends it",
    # The byte SMSG_INITIAL_SPELLS opens with. Player::SendInitialSpells writes
    # a literal uint8(0) and has no other send site, so there is no talent
    # group in it to read. GetActiveTalentGroup is answered from the talent
    # packets, which do carry one.
    ("InitialSpellsData", "talentSpec"): "AzerothCore writes a literal zero",
    # "1 = show window", and the one place that builds SMSG_SHOWTAXINODES
    # always writes 1 - the packet arriving *is* the instruction to open it.
    ("ShowTaxiNodesData", "windowInfo"): "the only send site always writes 1",
    # The charter list a registrar sends. Both of these are on the wire and
    # neither has a consumer, and the reason is the same for both: the arena
    # registrar's three tabs are told apart by the charter's *item name* -
    # "Arena Team Charter (2v2)" and its siblings - which GetPetitionItemInfo
    # already answers from the item query. The type word would say the same
    # thing a second time.
    #
    # The signature requirement is not shown at a registrar at all. It appears
    # on the charter once you hold one, and that comes from SMSG_PETITION_SHOW,
    # which this client does read - signaturesRequired is filled from it, and
    # was the fix that stopped every charter reading "x / 9".
    ("PetitionShowlistData", "charterType"): "the item name already says which team size",
    ("PetitionShowlistData", "requiredSigs"): "shown from SMSG_PETITION_SHOW, not the list",
    # Same field on the per-charter struct inside that list, for the same
    # reason.
    ("Charter", "requiredSigs"): "shown from SMSG_PETITION_SHOW, not the list",
}


def structs_and_accessors():
    """{struct: [fields]}, {field: {inline accessor names}}"""
    text = HDR.read_text(errors="ignore")
    fields, accessors = {}, collections.defaultdict(set)
    for m in re.finditer(r"\bstruct\s+(\w+)\s*\{(?:\.\w+\s*=\s*)?", text):
        name, i, depth = m.group(1), m.end(), 1
        while i < len(text) and depth:
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
            i += 1
        body = text[m.end():i - 1]
        # Short names are too common as ordinary words to count mentions of.
        fields[name] = [f for f in re.findall(
            r"^\s+(?:[\w:<>,\s\*&]+?)\b(\w+)\s*(?:=[^;]*)?;", body, re.M) if len(f) >= 4]
        for am in re.finditer(r"\b(\w+)\s*\([^)]*\)\s*(?:const\s*)?\{([^{}]*)\}", body):
            for f in fields[name]:
                if re.search(r"\b" + re.escape(f) + r"\b", am.group(2)):
                    accessors[f].add(am.group(1))
    return fields, accessors


def mentions(skip=None):
    """(parser counts, everywhere-else counts) - the header is neither."""
    parser, other = collections.Counter(), collections.Counter()
    sources = list((ROOT / "src").rglob("*.cpp")) + list((ROOT / "include").rglob("*.hpp"))
    for p in sources:
        if p.name == skip or p.resolve() == HDR.resolve():
            continue
        # A name that appears only in a comment is not a read.
        s = re.sub(r"//[^\n]*|/\*.*?\*/", "", p.read_text(errors="ignore"), flags=re.S)
        tgt = parser if PARSER.search(str(p)) else other
        for w in set(re.findall(r"\b\w+\b", s)):
            tgt[w] += s.count(w)
    return parser, other


def unread(fields, accessors, skip=None):
    parser, other = mentions(skip)
    plain, via = [], []
    for sname, fs in fields.items():
        for f in fs:
            if other[f] or not parser[f]:
                continue
            reached = sorted(a for a in accessors.get(f, ()) if other[a])
            (via if reached else plain).append((sname, f, tuple(reached)))
    # A struct declared twice - the same name under two #if branches, say -
    # yields the same field twice and would read as two findings.
    return sorted(set(plain)), sorted(set(via))


def main():
    fields, accessors = structs_and_accessors()

    # The canary. GetLootSlotInfo is the one reader of LootItem.lootSlotType,
    # so hiding its file must bring that field back. If it does not, the
    # comparison is broken and the empty report below means nothing - which is
    # precisely how the first version of this passed while seeing nothing.
    hidden, _ = unread(fields, accessors, skip="lua_inventory_api.cpp")
    if not any(f == "lootSlotType" for _, f, _ in hidden):
        print("  CANARY FAILED: hiding the one reader did not surface the field.")
        print("  Every count below is meaningless. Do not believe the zero.\n")
        return 1

    plain, via = unread(fields, accessors)
    settled = [r for r in plain if (r[0], r[1]) in EXPECTED]
    rows = [r for r in plain if (r[0], r[1]) not in EXPECTED]

    print(f"{len(fields)} packet structs, {len(via)} field(s) reached through an "
          f"accessor, {len(settled)} settled\n")
    print(f"{len(rows)} parsed and never read:\n")
    for sname, f, _ in sorted(rows):
        print(f"  {sname:34} {f}")
    if not rows:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
