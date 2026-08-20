#!/usr/bin/env python3
"""Run the fast sweeps and fail if any of them gets worse.

A sweep only helps on the day someone runs it. Every fault these catch was
found by hand at least once, and each is the kind that raises nothing, logs
nothing and fails no test - a panel drawn twice, a binding answering short, a
name in a manifest that resolves to nothing, a chunk of bootstrap Lua replacing
the binding underneath it. Left to a person remembering, they come back.

    tools/sweep_guard.py          # report and exit non-zero on a regression
    tools/sweep_guard.py --list   # show the ceilings without running anything

Each entry pins a ceiling rather than an exact figure, so a fix that lowers a
count passes and is meant to be followed by lowering the ceiling here - a
ratchet, not a snapshot. Where the honest answer is none, the ceiling is zero
and lowering it is not possible.

These run in about a minute together, which is why they are the ones wired
into the build; the slower reports - element readiness, the unbound global
scan, the ungated-draw walk - stay manual. The line here used to say six of
them in under three seconds, and had said so for long enough that the count was
out by a factor of five: a comment about how much a thing costs stops being
true the moment someone adds to it, and this one is worth keeping honest
because it is the argument for what belongs here.
"""
import argparse
import pathlib
import os
import re
import subprocess
import sys

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent

# tool, pattern capturing the count, ceiling, what the count means
CHECKS = [
    # The settings screens are generated from one schema; saving them is not.
    # A setting added to that list appears in both windows, answers its value,
    # applies correctly - and is gone at the next login if nobody wrote it out.
    # Nothing raises, and every other test passes.
    ("settings_persist_check.py",
     r"^(\d+) no field behind it", 0,
     "settings in the schema with no field to store"),
    ("settings_persist_check.py",
     r"^(\d+) never written to the config file", 0,
     "settings that reset at every login"),
    # FrameXML asks whether a frame has a method and means it. A method on the
    # metatable every frame shares makes the question unanswerable, and the
    # branch meant for the few controls that define their own is taken by all
    # of them. This is how every options slider came to read its value off a
    # status bar.
    ("framexml_presence_test_check.py",
     r"^(\d+) presence test\(s\) the shared metatable always passes", 0,
     "presence tests the shared metatable always passes"),
    # The three sound switches are applied by zeroing a channel, and zeroing
    # has no inverse - anything they silence has to be written again by
    # applyAudioVolumes or it stays silent for good. The sound-effects switch
    # zeroed nine managers and eight were restored: the player voice had its
    # enabled flag set and never its volume, so a character lost their speech
    # the first time that switch was turned off.
    ("audio_channel_restore_check.py",
     r"^(\d+) silenced and never turned back up", 0,
     "sound channels silenced with nothing to turn them back up"),
    # A renderer builds its pipeline twice - once in initialize, once in
    # recreatePipelines when the swapchain or the sample count changes - and
    # the two have to describe the same thing. Celestial's did not: no depth
    # test in one and a depth test in the other, so the sun and moon changed
    # behaviour on the first resolution change, long after anyone was looking
    # at the code that caused it.
    ("pipeline_recreate_check.py",
     r"^(\d+) that rebuild a different one", 0,
     "renderers that rebuild a different pipeline than they built"),
    ("settings_persist_check.py",
     r"^(\d+) written but never read back", 0,
     "settings saved to the config file and never loaded"),
    # GameHandler was split into ChatHandler, SocialHandler and the rest, and
    # every move left the original method where it was. Most became forwarders
    # and are still dispatched. Three were not: a second whole implementation
    # of SMSG_NOTIFICATION and SMSG_QUERY_TIME_RESPONSE, and a forwarder no
    # table names. The dead-symbol sweep passes all three, because it matches
    # on the name and the name does have a caller - just not that copy's.
    # register_test_target adds the target to the list the ASAN and UBSan build
    # iterates, and links ws2_32 on Windows. Six targets called add_test and
    # not it: green under ctest, and absent from every sanitised run, which is
    # the one place a test exists to be run.
    # GLSL has no linker here: a function two shaders need is written into
    # both. Seven bodies are duplicated across the seventy-eight shaders, and a
    # copy that drifts compiles and raises nothing - it shades one kind of
    # surface unlike the others, which reads as an art problem.
    # tools/ is a hundred and eight scripts and nothing scanned them. Four
    # parser functions were duplicated across sweeps, two character for
    # character. A sweep with a private copy of a parser does not fail when the
    # copy stops recognising something - it reports fewer findings and goes
    # green for a reason nobody looks at.
    ("tool_duplication_check.py",
     r"^(\d+) function\(s\) written twice across scripts", 0,
     "parsers a sweep keeps its own copy of"),
    ("shader_function_check.py",
     r"^(\d+) whose copies no longer agree", 0,
     "shader functions whose copies no longer agree"),
    ("test_registration_check.py",
     r"^(\d+) that the sanitiser build never sees", 0,
     "tests ctest runs that the sanitiser build never sees"),
    ("handler_twin_check.py",
     r"^(\d+) copy that nothing reaches", 0,
     "handlers left behind in the class they were moved out of"),
    # The block scan reads a twelve-line window and is at zero, and most
    # functions here are shorter than that. Comparing whole bodies instead
    # found five pairs it could not see, including two renderers that had
    # disagreed about a rotation order for a long time.
    ("function_similarity_check.py",
     r"^(\d+) pair\(s\) that are one function written twice", 0,
     "one function written twice under two names"),
    # The other end of copying: not a whole function repeated but one line
    # copied down a run of axes with an edit left unfinished. Every line reads
    # correctly on its own, which is why reading them confirms nothing; what
    # gives it away is the column that stops walking with the others.
    ("copy_paste_axis_check.py",
     r"^(\d+) run\(s\) where one axis disagrees", 0,
     "axes written out per line with one left behind"),
    # A layout column that exists and holds the wrong field. dbc_layout_check
    # asks only whether the column is in the file, which is the easy half: 71
    # is a valid index in a 173-field Spell.dbc and was the wrong one, and no
    # profession window opened on Classic or TBC for as long as the layouts
    # existed. A row id means the same row in every expansion's copy, so the
    # declared column has to match the reference file's better than its
    # neighbours do.
    ("dbc_column_agreement_check.py",
     r"^(\d+) column\(s\) whose neighbour matches", 0,
     "layout columns a neighbouring column matches better"),
    # A diagnostic meant to fire a few times whose counter is advanced under a
    # narrower condition than the one letting it through. One logged every
    # frame of every session - Hellfire has one light volume in range and the
    # counter only moved when a third was pushed - and it cost a formatted
    # write per frame on the main thread, only while the player moved, because
    # the logger folds a line identical to the one before it.
    ("bounded_log_check.py",
     r"^(\d+) bounded log\(s\) whose counter", 0,
     "bounded diagnostics whose counter may never advance"),
    # A sound loaded at every start-up that nothing can trigger. Found 34 of
    # these once, 109 wav files between them, in three kinds: a duplicate of a
    # general path, a fallback nobody could reach, and a feature never
    # finished. They wanted deleting, wiring and deciding respectively - so a
    # hit here is a question rather than a verdict, and the ceiling is zero
    # because every one of them was answerable.
    ("unused_sample_check.py",
     r"^(\d+) sample collection\(s\) loaded and never played", 0,
     "sound samples read from disk that nothing plays"),
    # A checkbox that saves its CVar, reads it back, and changes nothing looks
    # exactly like one that works - it remembers what you chose. 69 of the 198
    # controls in the option panels had no reader at all; 14 of those are
    # honestly greyed with a reason, and this counts the rest. The ceiling
    # comes down as they are implemented or greyed, and must never go up: a new
    # control wired to nothing is the thing being watched for.
    ("dead_setting_check.py",
     r"^settings with no reader and still on a panel: (\d+) of", 1,
     "option panel controls whose CVar nothing reads"),
    # Both halves still write to the chat window. The handler adds a line and
    # fires the event; chatframe.lua's own branch formats the same fact from
    # the event and adds it too, and the player reads it twice.
    # glm arrives with an imported target, and Linux has a system copy that
    # hides a missing link entirely. Every one of these built green here and
    # failed on macOS, one per CI run, because the build stops at the first.
    # Clang calls these -Wunused-private-field and only some versions do: the
    # Windows CI image reported two that clang 18 on Linux does not, so a local
    # build is not a gate for this class. The ceiling is where the count stands
    # today, not an endorsement of it; it exists so the number can only fall.
    # Only fails where the macro is actually passed, so a Windows-only build
    # error for something greppable on any host.
    # Three Windows failures in a row were a POSIX-only function called
    # directly in one place among several that had all remembered the #ifdef.
    ("posix_only_check.py",
     r"^(\d+) called directly instead", 0,
     "POSIX-only calls that break the Windows build"),
    ("redefined_macro_check.py",
     r"^(\d+) redefined without an #ifndef", 0,
     "macros the build defines, redefined unguarded"),
    ("unused_member_check.py",
     r"^(\d+) members stored and never read", 43,
     "class members stored and never read"),
    # The subset clang's -Wunused-private-field rejects outright, which is a
    # failed Windows build rather than debt. Zero, and it stays there.
    ("unused_member_check.py",
     r"^(\d+) private and never referenced", 0,
     "private members nothing mentions, which fail the Windows build"),
    ("test_glm_link_check.py",
     r"^(\d+) reach glm without it", 0,
     "test targets that build on Linux and fail on macOS"),
    ("chat_line_twice_check.py",
     r"^(\d+) of them written a second time", 0,
     "chat lines the interface already writes"),
    ("handover_halves_check.py",
     r"^(\d+) with no frameXmlOwns gate", 0,
     "elements this client keeps drawing after handing them over"),
    ("handover_halves_check.py",
     r"^(\d+) with no suppression entry", 0,
     "elements FrameXML draws while this client still owns them"),
    ("handover_check.py",
     r"^(\d+) call\(s\) naming nothing that exists", 0,
     "interface commands naming a function that does not exist"),
    ("handover_check.py",
     r"^(\d+) action\(s\) acted on in more than one file", 0,
     "keys driving the interface from two places, which cancel out"),
    ("framexml_load_check.py",
     r"^(\d+) that resolve to nothing", 0,
     "manifest entries and script references pointing at no file"),
    # Twenty, and the rise is the sweep seeing more rather than the client
    # doing worse - the same blind spot the argument sweep had: only bindings
    # written as *named* functions were matched, and a lambda has no name, so
    # more than half were never counted. Eight rows appeared and one was real:
    # GetAuctionSellItemInfo answered six values where the sell tab unpacks
    # nine and then does `if totalCount > 1` the moment an item is dropped in,
    # so it raised on a nil instead of showing a slot without stack controls.
    #
    # Of the rest, five are the Battle.net family, which has no server behind it
    # in 3.3.5a; GetGuildTabardFileNames answers nothing and its callers set
    # textures, where nil reads as an empty slot; and GetGossipOptions returns a
    # count it computes, which this sweep reads as zero - the known shape named
    # in its docstring.
    ("framexml_short_returns.py",
     r"^(\d+) binding\(s\) may return short", 0,
     "bindings answering fewer values than the interface unpacks"),
    ("misleading_indent_check.py",
     r"^(\d+) statement\(s\) dressed as though", 0,
     "statements dressed as though a braceless if guarded them"),
    # Seventeen, each read on 2026-08-05 and listed in the tool's docstring.
    # Three are XML namespace boilerplate rather than attributes at all, and
    # the rest divide into "no mechanism here" (texture tiling needs a REPEAT
    # sampler and the UI's is shared and CLAMP_TO_EDGE) and "no consequence"
    # (build metadata on Binding, an async-load hint). The one that came out
    # was motionScriptsWhileDisabled, which was not inert: nothing here ever
    # suppressed motion scripts, so every greyed control answered the mouse.
    ("declared_vs_read_check.py",
     r"attributes declared, (\d+) the emitter never names", 16,
     "XML attributes the emitter never reads"),
    # The other half. An attribute the emitter ignores is a wrong value on a
    # frame; an element it ignores is a frame, region or animation that never
    # exists at all, and nothing says so. Zero, and it has to stay zero - the
    # one case that ever failed was FrameXML's own <Fontstring>, which is a
    # typo for FontString and which the emitter folds before comparing.
    ("declared_vs_read_check.py",
     r"element\(s\) in the files that load, (\d+) the emitter never names", 0,
     "XML elements the emitter never builds"),
    # Eight, not zero, and the number went up because the check started
    # working. It used to accept any "OnX" literal anywhere in src, and the
    # emitter carries a table naming every script type it knows a signature
    # for - so every type read as fired and this reported zero for ever, while
    # OnCursorChanged sat unfired and every multi-line edit box in the
    # interface raised on the first keystroke.
    #
    # Two, and both named: the chat box's input language, which the binding
    # beside it says outright that nothing fires yet, and the tooltip's default
    # anchor. Everything else that once appeared here was the sweep's own
    # blindness - see the comments in the check.
    ("declared_vs_read_check.py",
     r"^(\d+) script type\(s\) declared and never fired", 2,
     "script handlers FrameXML declares that nothing fires"),
    ("declared_vs_read_check.py",
     r"sound names asked for, (\d+) with no hand-written mapping", 24,
     "UI sounds with no hand-written mapping (the dbc answers most of them)"),
    # "Read as off" is exact, and was checked rather than assumed: pushCvarDefault
    # ends in a catch-all answering "0", so an unlisted CVar comes back "0" and
    # GetCVarBool false. Nothing here reads nil, and nothing raises.
    #
    # Which makes the remainder safe rather than urgent. A setting whose real
    # default is off already behaves correctly; only one whose default is on is
    # wrong, and turning any of these on needs a source for the value. The
    # uvarInfo table was such a source and its 27 have been taken; for the rest
    # there is no statement of the default anywhere in the tree, and a number
    # invented here would be a behaviour change wearing the clothes of a fix.
    #
    # Two things do read as off wrongly and neither is reachable: the CVars for
    # movie recording and for voice chat, both features this client does not
    # have. MacOptionsFrame was shown and updated to check the first of those -
    # it draws without raising, so the arithmetic worry there is not real.
    ("declared_vs_read_check.py",
     r"CVars named, (\d+) the client never answers", 41,
     "CVars the client never answers, so they read as off"),
    # The half of that list that raises rather than reading as off. A CVar the
    # interface only tests survives answering nothing - the branch behind it
    # does not run. One fed to tonumber() and then to arithmetic does not:
    # watchframe.lua sets WATCHFRAME_FILTER_TYPE from
    # tonumber(GetCVar("trackerFilter")) on VARIABLES_LOADED and then calls
    # bit.band on it, so the quest tracker worked until the login event meant to
    # configure it and went down on its next update, every session. The world
    # map had the same shape in WorldMapFrame_SetOpacity.
    #
    # One left: MovieRecordingCompression, behind `if (not IsMacClient()) then
    # return` in macoptionsframe.lua, and IsMacClient answers false.
    ("declared_vs_read_check.py",
     r"^(\d+) CVar\(s\) with no default that the interface does arithmetic", 1,
     "CVars with no default that the interface does arithmetic on"),
    # Zero now, because the one that will never agree is named in the tool with
    # why instead of counted here. DEFAULT_CHAT_FRAME's bootstrap value is a
    # stand-in for before the interface loads and FrameXML replaces it on
    # purpose; what that cost is handled by the redirect in
    # AddonManager::loadFrameXml rather than by making the two agree. A count
    # of one could have hidden a second disagreement behind it.
    ("declared_vs_read_check.py",
     r"^(\d+) constant\(s\) set in both places", 0,
     "constants the bootstrap and the interface disagree about"),
    # Both arms are zero, and what used to hold them up is now named in the
    # tool with the reason rather than counted here - a count of one or four
    # cannot tell a decision from a new fault arriving as an old one leaves.
    ("framexml_event_arity.py",
     r"^(\d+) fired with fewer arguments than a handler reads", 0,
     "events fired with fewer arguments than a handler reads"),
    ("framexml_event_arity.py",
     r"^(\d+) fired from several places with differing counts", 0,
     "events whose argument count depends on which path fired them"),
    # Five, and every one is unreachable: the event that shows its dialog is
    # not fired anywhere under src/. LEVEL_GRANT_PROPOSED is Recruit-a-Friend
    # level granting, END_BOUND_TRADEABLE and END_REFUND end a refund window,
    # and TRADE_REPLACE_ENCHANT is the trade-window twin of REPLACE_ENCHANT -
    # which *is* fired, from inventory_handler, and whose verb is bound.
    #
    # It was twenty-five. The rest went as the bind-confirmation family and the
    # socketing prompts were implemented, and the ceiling stayed at twenty-five
    # until 2026-08-05, permitting twenty silent regressions.
    #
    # The ceiling is for the day one of those four events starts being fired: a
    # popup that opens with an unbound accept is a player pressing a button and
    # getting an error. That day came for CONFIRM_LOOT_ROLL, and in the useful
    # direction - it is raised by the *client*, not the server, so nothing here
    # raised it and Need on a bind-on-pickup item bound it with no warning.
    # Checking whether a popup is reachable means asking which event shows it
    # and whether this client fires that event, not whether the verb is bound.
    ("staticpopup_verbs_check.py",
     r"^(\d+) name\(s\) a popup button calls and nothing answers", 0,
     "names a static popup's buttons call that nothing answers"),
    # The three wire-shape checks. Each is a fault that no test catches: a
    # request the server drops on the floor, a reply read at the wrong offsets,
    # a guard that stops a handler running at all.
    ("cmsg_size_check.py",
     r"^(\d+) request\(s\) shorter than the server reads", 0,
     "requests shorter than the server reads, which it drops"),
    ("cmsg_size_check.py",
     r"^(\d+) request\(s\) written in a different shape", 0,
     "requests written in a different field order from the server's"),
    ("packet_layout_check.py",
     r"^(\d+) packet\(s\) read in a different shape", 0,
     "replies read at different offsets from the ones written"),
    ("packet_size_check.py",
     r"^(\d+) guard\(s\) longer than the packet", 0,
     "guards longer than the packet, so the handler never runs"),
    # Six, and five of them are right: scripts and registered events live on
    # the Lua table and always will, and GetName reads a name set once at
    # creation. The ceiling is for the seventh. SetParent sat in this list
    # writing __parent while layout went on using the widget's, and GetCenter
    # read a field that only a dead SetPoint had ever written.
    ("widget_field_check.py",
     r"^(\d+) method\(s\) touch only a Lua field", 0,
     "frame methods writing a Lua field where the widget is what is read"),
    # Nineteen, none of them reachable: the login splash, the tic-tac-toe
    # minigame, and one talent-frame background behind a branch that cannot run
    # - SELECTEDSPEC_DISPLAYTYPE is "GOLD_INSIDE" and the texture is only asked
    # for by the two "PUSHED_OUT" spellings. The ceiling is for the twentieth. A
    # missing texture raises nothing: the frame is built, laid out and drawn,
    # and the part that should have art is simply absent.
    ("framexml_art_check.py",
     r"^(\d+) not in this install", 1,
     "art the interface asks for that this install does not have"),
    # CVAR_UPDATE carries the CVar's label, not its name, and the two are
    # different spellings - so a mapping that cannot produce a label the
    # interface tests for is a branch that can never be taken. Silently: a
    # string that is not equal to another string is not an error.
    ("framexml_cvar_label_check.py",
     r"^(\d+) that no CVar name here would produce", 1,
     "CVAR_UPDATE labels the interface tests for that nothing can produce"),
    # The same question the event-arity sweeps ask, one layer down: an inline
    # <OnX> body is a function whose parameter list the emitter decides from
    # the script's name, and a body naming something that list does not carry
    # reads a global, finds nothing and carries on with nil. Two remain and
    # both are Blizzard's own deliberate nil, guarded on the far side.
    ("framexml_script_args.py",
     r"^(\d+) body/signature disagreement", 0,
     "handler bodies reading an argument their signature does not carry"),
    # A dozen things are driven by finding a FrameXML frame by name and
    # handing it something - the minimap and world map are told where to be,
    # every portrait and model frame is handed an image rendered for it. A name
    # matching nothing answers null: no error, no warning, no picture, and a
    # typo looks exactly like a frame that was never built. Zero, because there
    # is no reason to look up a name the interface does not have.
    ("framexml_lookup_names_check.py",
     r"^(\d+) looked up that the interface does not declare", 0,
     "frames this client looks up by a name the interface does not declare"),
    # Unit bindings that never look at the unit they were asked about and
    # answer from the player. A number belonging to the wrong character is the
    # hardest kind of wrong to see - nothing empty, nothing zero, nothing
    # raised. UnitStat, UnitResistance, UnitArmor, UnitAttackPower, UnitDamage
    # and UnitFactionGroup all sat here, listing a hunter's own figures as the
    # pet's and putting an Alliance badge over a Horde target;
    # SetInventoryItem did the same on the inspect paperdoll.
    #
    # Seven, each read once. Five are the player's sheet alone, the pet tab
    # having no ranged or defence line - UnitDefense, UnitRangedAttack,
    # UnitRangedAttackPower, UnitRangedDamage, UnitAttackBothHands. The other
    # two are the check's own blind spots, named in its docstring: IsUnitOnQuest
    # takes the unit second, and UnitPlayerOrPetInRaid delegates to a binding
    # that does resolve.
    #
    # UnitControllingVehicle left when the *named* bodies were brace-matched as
    # well as the inline ones - a one-line function has no closing brace at the
    # start of a line, so the non-greedy form ran on and gave it the next
    # function's calls.
    #
    # Was ten, and the two that left were never really here. The inline-lambda
    # body was found by looking for a closing "}}" at a fixed indent, which only
    # a multi-line lambda has, so a one-line one was invisible and a multi-line
    # one's body ran on to the next "}}" it could find - swallowing whatever lay
    # between and inheriting its gh->getX() calls. GetUnitHealthModifier,
    # UnitIsSameServer and SetAchievementComparisonUnit were reported for code
    # that was not theirs, and which bindings appeared depended on how the ones
    # above them happened to be formatted: reformatting GetStatistic changed the
    # count. Braces are matched now, and the one-line form was seen to be caught
    # before this number was trusted.
    ("unit_argument_check.py",
     r"^(\d+) unit binding\(s\) that never look at their unit", 0,
     "unit bindings answering from the player whatever they were asked"),
    # Requests the server reads off the wire and throws away - an opcode
    # registered Handle_NULL. Two, and both are accounted for:
    # CMSG_SUSPEND_COMMS_ACK is an acknowledgement the server has no use for,
    # and CMSG_PET_UNLEARN_TALENTS has no live opcode to replace it - a pet
    # talent wipe is a spell, the way the player's spec switch turned out to
    # be. A third row means a request that leaves and changes nothing, which
    # is the quietest failure a request has: nothing malformed, nothing
    # logged, no size or layout check disturbed. The difficulty change and the
    # ready-check answer both sat here.
    ("discarded_request_check.py",
     r"^(\d+) that the server reads and discards", 2,
     "requests the server reads and discards"),
    # The same table read backwards: a message the server sends that this
    # client names and never handles. Judged by whether the server *builds* one
    # anywhere outside its own opcode table - a name kept only for recognition
    # is not a gap, and one in a WorldPacket constructor is a packet arriving
    # here and being dropped. Three were live on 2026-08-06: the hover,
    # feather-fall and water-walk broadcasts, missing from the relay list their
    # siblings were already in. Zero, and taking one back out reports it. 4s.
    ("discarded_request_check.py",
     r"^(\d+) that the server actually builds", 0,
     "server messages this client is sent and never handles"),
    # Update-field indices against the server's own UpdateFields.h. Zero, and it
    # has to stay zero: a wrong index reads whatever sits at that slot and the
    # value is simply wrong forever, with no error anywhere. Five were wrong
    # when this was written - UNIT_FIELD_BYTES_1 and UNIT_DYNAMIC_FLAGS at 137
    # and 147, which are inside the unit block and so look right, against the
    # server's 74 and 79; and the chosen title and both PvP currencies past
    # PLAYER_END, where nothing can arrive. WotLK only: it is the only server
    # here, and the other expansions' files are unverifiable from it.
    ("update_field_check.py",
     r"^(\d+) disagree with the server's own header", 0,
     "update-field indices disagreeing with the server's header"),
    # DBC field indices naming a column the file does not have. One:
    # CharacterFacialHairStyles.Geoset200 = 8. The layouts describe the stock
    # nine-column file, which is a real shape and the right thing for them to
    # describe; the eight-column file both installs here carry is handled by
    # detectFacialHairFields, which picks 3-5 on the field count. Lower this
    # only by making that decision somewhere the JSON can express, and read a
    # new row as a column being read from padding - that one was drawing no
    # facial hair at all and saying nothing.
    #
    # One rather than four because the check now scores the four layouts
    # against the files and reads only the best-fitting one. There is a single
    # set of DBCs here; checking a Classic layout against WotLK data reported
    # three expansions' worth of noise that looked exactly like findings.
    ("dbc_layout_check.py",
     r"^(\d+) field\(s\) naming a column the file does not have", 1,
     "DBC field indices naming a column the file does not have"),
    # Packet handlers that change this client's own model, tell the player in
    # chat, and tell the interface nothing - the shape that produces a bug
    # correct after a relog and wrong until then. Twenty, each read once: what
    # they write is bookkeeping nothing draws.
    #
    # The count rose from fourteen when the sweep learned to read named handler
    # methods as well as inline lambdas - a dispatch entry is often one line
    # calling handleFoo, and reading only the lambda sees a body that calls one
    # function. Four real ones have been fixed: the equipment manager's new
    # set, the withdrawn summon dialog, a dead pet's frame and ability bar, and
    # the flight map left open for the whole flight.
    ("handler_announce_check.py",
     r"^(\d+) that tell the player and not the interface", 0,
     "handlers that change state and announce nothing"),
    # Packet fields the parser fills in that no line outside the parser reads.
    # The mirror of handler_announce_check: that one asks what a handler fails
    # to pass on, this asks what the wire carried that nothing collected.
    #
    # Three, all in the charter-purchase path: the signature counts and type on
    # the charters a vendor lists, which that list does not display - the
    # petition being signed reads its own requirement from a different struct.
    #
    # It was four. AuctionMailInvoice::ownerGuidLow came out when the auction
    # mail invoice was wired to FrameXML: parseAuctionMailBody had been
    # decoding those bodies all along and only this client's own mail window
    # read them, so handing mail over left the breakdown behind and a sale
    # arrived as a letter with the raw colon-separated body in it. A field
    # nobody reads usually means a branch nobody runs.
    #
    # Five were answered the day it was written and
    # both were live - a loot row the player cannot take drawing as ordinary
    # loot, and a group invite the server had already refused raising the
    # accept popup. Both sat next to a comment saying the data was unavailable,
    # which is the shape this exists to catch.
    #
    # The tool carries its own canary, because two separate mistakes make it
    # report a clean zero while seeing nothing at all.
    ("parsed_never_read_check.py",
     r"^(\d+) parsed and never read", 3,
     "packet fields stored by the parser and read by nobody"),
    # The mirror of the row above: a field every reader agrees on and no writer
    # ever fills, so they all agree on the declaration's initialiser. Zero,
    # because the one that was found - a charter's signature requirement,
    # permanently nine - is fixed and nothing should replace it.
    #
    # Three exceptions are named in the tool rather than counted here, all of
    # them written somewhere a member assignment does not appear: a reference
    # out-parameter, and a whole-struct assignment. The tool carries a canary,
    # because two of its three write-forms were added only after an empty
    # report turned out to be an empty *sweep*.
    ("read_never_written_check.py",
     r"^(\d+) read and never written", 0,
     "struct fields with readers and no writer"),
    # Frames the interface declares that the emitter does not create. Runs the
    # real emitter over each file and looks for the name in what came out,
    # which is the one question about it that reading either side cannot
    # settle.
    #
    # Zero across 172 files and 2369 frames. It matters because a dropped frame
    # raises nothing: the handlers that touch it die on a nil index somewhere
    # else entirely, and what the player sees is a panel that is present and
    # does nothing - the shape of several reports this week.
    #
    # Needs the framexml_emit target; it says so and stops rather than
    # reporting a clean zero it did not earn.
    ("framexml_frame_emitted_check.py",
     r"^(\d+) file\(s\) with frames the emitter does not create", 0,
     "frames declared in XML that the emitter never creates"),
    # Top-level FrameXML frames named nowhere in framexml_takeover.cpp -
    # neither handed over nor suppressed.
    #
    # This comment used to say all forty-four had been checked against this
    # client's own UI and none had a counterpart. That was wrong, and wrong in
    # the way a claim written once and cited afterwards usually is: the check
    # it describes was a search for look-alike windows, and what makes a
    # duplicate is a shared *trigger*. Three of the forty-four were charter
    # windows raised by three events this client fires, beside two popups
    # social_panel.cpp draws from the same packets. They are
    # UiElement::Petition now.
    #
    # So the ceiling guards a list that has been read the right way once, on
    # 2026-08-05, with the result written into the tool's docstring: fifteen
    # dormant, eighteen opened by a control in FrameXML's own interface, four
    # waiting on an event nothing fires. The thirty-eighth is the one to look
    # at. The tool's own blind spot is frames built by CreateFrame, which its
    # docstring measures and lists.
    # Bindings answering a boolean or nil where FrameXML compares a number.
    # Never in this list until 2026-08-05, which is the whole reason its nil
    # arm could be added and be hollow at the same time: nothing ran it.
    # Two standing rows, both read and both written into its docstring -
    # an unreachable debug reader and a Wintergrasp timer whose nil is
    # `and`-guarded.
    # A global FrameXML calls that exists here only as a widget method. The
    # unbound-global sweep cannot see this by construction - methods and
    # globals register the same way, so a method makes the global read as
    # answered while it stays nil. GetText was that for months, raising every
    # time the reputation list opened. Zero is the only acceptable number and
    # the tool carries three canaries so a zero means something.
    # This client's own slash commands, and whether FrameXML's chat can reach
    # them. Zero with the bridge in addon_manager, seventy-one without - so
    # the number answers the real question rather than standing in for it, and
    # deleting the bridge is what makes it fail. Verified that way.
    # Verbs this client's own windows can reach and FrameXML cannot. The
    # question that found the slash-command registry, so it is worth running
    # rather than remembering - 2.7s. Sixty-six, all triaged in the tool:
    # callback wiring, the glue screen, the 3D world, five with a bound
    # FrameXML equivalent, and window state FrameXML replaces whole. The
    # sixty-seventh is the one to look at.
    ("framexml_unreachable_verbs.py",
     r"^(\d+) verbs this client's own windows can reach", 66,
     "verbs only this client's own windows could reach"),
    # Emote tokens FrameXML can hand DoEmote that it cannot answer. One,
    # named "unused", which is a placeholder in FrameXML's own list. It was
    # two hundred and twenty-one until 2026-08-05, when DoEmote stopped
    # answering from a map written by hand and started reading EmotesText.dbc
    # through EmoteRegistry - the same table this client's own chat had been
    # reading directly all along, which is why nobody noticed until chat was
    # handed over and DoEmote became the only route.
    # Chat types FrameXML can hand SendChatMessage that it does not map. One,
    # REPLY, and it is correct: processChatType rewrites /r to WHISPER before
    # anything is sent, so it cannot arrive. The binding mapped eight of
    # thirteen until 2026-08-05 and *defaulted to SAY*, so every numbered
    # channel line, every raid warning and every custom emote was said out
    # loud to whoever was standing nearby. It refuses an unknown type now.
    # Sweeps that cannot run at all. Not their numbers - this runs the ones
    # sweep_guard does not, and reports any that raise or exit non-zero.
    #
    # Two broke on 2026-08-05 from a single edit: the candidates tier lost its
    # `for (const char* name : {...})` loop and two tools parsed that loop and
    # called .group(1) on the None they got. handover_halves_check is in this
    # list, so ctest caught it in under a minute; framexml_live_stubs is not,
    # and it sat crashing until someone happened to run it. 18s.
    # Bindings that answer something plausible and do nothing, reachable from
    # an element FrameXML draws. Thirty-nine, and almost all are systems that
    # are genuinely absent - voice, Battle.net, movie recording, the debug zone
    # map, arena opponents, WotLK's non-existent skill points. Two were read
    # properly on 2026-08-05 and left: GetBattlefieldInstanceRunTime, whose
    # value is in no packet this client is sent, and the container sell-cursor
    # pair, which is cosmetic.
    #
    # The ceiling is for the next one. A stub added on a handed-over element is
    # the shape DoEmote had - plausible, silent, and invisible until the panel
    # that needs it is the only route. 8s.
    #
    # A hundred and ninety-five now, and the jump is this sweep learning to see
    # a stub written out in place. It recognised only the named shared ones -
    # lua_ReturnNil and its family - so a lambda answering a literal in the
    # table counted as an implementation. Which is what a stub is: the test is
    # now that stripping the pushes and the return leaves nothing that calls
    # anything, arrived at after asking the question the other way round marked
    # GetActionBarPage, GetSelectedFaction and GetSelectedSkill, all three of
    # which do consult something - a Lua global, a C++ static - just not the
    # game handler.
    #
    # The thirty-nine above were each read. The hundred and fifty-six that
    # joined them have not been, and saying so is the point of writing it here.
    # Two are worth naming: GetQuestLogCompletionText answers "" and
    # GetQuestLogRequiredMoney answers 0, both to the quest log and the tracker.
    # Those two are real, and answering them needs SMSG_QUEST_QUERY_RESPONSE
    # parsed further than it is - the quest log entry carries neither field, and
    # the packet that does carry them is the turn-in one, which arrives only
    # when the player is already standing at the NPC.
    ("framexml_live_stubs.py",
     r"^\d+ bindings, \d+ of them stubs, (\d+) reached from a handed-over element", 128,
     "stubs reachable from an element FrameXML draws"),
    # Bindings that read fewer arguments than the interface passes. The only
    # sweep here that asks whether an answer used everything it was told -
    # every other one asks whether a name is answered, and these names all are.
    #
    # Seven faults on 2026-08-05, four of which *acted* on the wrong thing:
    # dragging a pet spell put a player spell on the bar, clicking a pet talent
    # spent a point in the player's tree, cancelling an aura by name raised
    # instead of cancelling, and click-casting on a unit frame cast on the
    # current target. None raised, and the icons were right in every one.
    #
    # Fifty-four, and the rise is the sweep seeing more rather than the client
    # doing worse: it matched only bindings written as named functions, so the
    # 738 registered as inline lambdas - more than half - were never asked the
    # question. Three faults came out of that half and are fixed; the twenty-two
    # rows it added are otherwise trailing optional flags of the kind already
    # here (exactMatch on StartDuel, FollowUnit, PromoteToLeader; showError on
    # CanInspect; includeAll on GetCategoryNumAchievements).
    #
    # Mostly genuinely optional arguments - the self-cast flag on UseAction and
    # its siblings, the show-realm flag on GetUnitName, the notify flag on
    # SetCVar - and three that are wrong and named here:
    # GetAttackPowerForStat needs a per-class coefficient table this client does
    # not have; StartAuction's numStacks would need a several-item request where
    # the builder writes one; DoEmote's target is a player *name*, and there is
    # no name-to-guid lookup here to resolve it with, so it emotes at the
    # current target. Each is wrong either way, and a guess would be harder to
    # notice than the gap. 4s.
    # Fifty-five, and the move from fifty-four is composition rather than
    # decay: the named-function bodies are brace-matched now, like the inline
    # ones, instead of running to the first "\n}" - which a one-line function
    # does not have, so it took the next function's body with it. Two rows were
    # never really there (SetOverrideBindingClick, ShowContainerSellCursor) and
    # three had been hidden behind swallowed text.
    #
    # Of those three: UnitBuff and UnitDebuff ignore a filter that is only ever
    # "RAID", and only when showCastableBuffs is on - which has no default and
    # so reads false. Answering it means knowing which buffs the player could
    # cast or dispel, which this client has no model of, so the list would be
    # wrong in a different way. GetContainerItemPurchaseInfo ignores the
    # isEquipped flag and would look an equipped item up in a bag, which costs
    # nothing while the refund window it reports is untracked either way.
    # Fifty-seven: PromoteToAssistant and DemoteAssistant joined when they
    # stopped being no-ops, and both ignore the same trailing exactMatch flag
    # PromoteToLeader beside them does. They match exactly, which is the
    # behaviour that flag asks for and the only value the unit popup passes.
    ("binding_arg_coverage_check.py",
     r"^(\d+) binding\(s\) read fewer arguments", 55,
     "bindings that ignore an argument the interface passes"),
    ("tools_run_check.py",
     r"^(\d+) that cannot run", 0,
     "sweeps that cannot run at all"),
    # A binding is registered two ways - as a named function, or as a lambda
    # written out in the table - and they are the same binding to Lua. Five
    # sweeps matched only the named form, so each asked its question of fewer
    # than half the bindings while reporting a number that read as all of them.
    # Fixing them on 2026-08-06 turned up a raising auction sell tab, a Create
    # All that made one item, a pet sent at the wrong target, three boolean
    # answers compared numerically, and a hundred and fifty-six uncounted
    # stubs. Zero, and one sweep was blinded again and seen to be caught before
    # that zero was believed.
    ("tools_run_check.py",
     r"^(\d+) that read only one of the two binding forms", 0,
     "sweeps that see under half the bindings they claim to check"),
    # A lazy loader that sets its "done" flag before checking the assets are
    # there disables itself for the session on one early call - no error, and
    # the panel is simply empty from then on. Zero, and the fault was put back
    # into ensureAchievementCategoriesLoaded and seen to trip it. Reported one
    # before it was taught to stop at the end of a function: a latch on the
    # last line of CharacterPreview::loadCreature was being read against the
    # opening guard of the next function down. 1s.
    ("lazy_load_latch_check.py",
     r"^(\d+) latch before checking that the assets exist", 0,
     "lazy loaders that latch before checking their assets are there"),
    # A frame named in the readiness tables that no file the loader reaches
    # declares. It reports NOT BUILT for ever, which reads as a fault in the
    # interface rather than in the list - and the list is where it is. Zero,
    # and a made-up name was put in and seen to trip it. 2s.
    ("framexml_promised_frames.py",
     r"^(\d+) promised by a table and declared nowhere", 0,
     "frames the readiness tables name that nothing declares"),
    # The mirror of the sweep above: not whether the answer used what it was
    # told, but whether it is spelled the way the caller looks it up. A token is
    # a table key, so a misspelling is a nil rather than a wrong value - the
    # rune bar's prefix came back nil from _G[""], and UnitRace's file name
    # asked for an asset with a space in it. Zero, and both faults were put back
    # and seen to trip it before that zero was believed. 1s.
    ("token_table_check.py",
     r"^(\d+) token\(s\) spelled so", 0,
     "token strings the interface cannot look up"),
    # Where the interface indexes a table with a binding's answer directly. A
    # reading list, not a verdict - the sweep can print the table's keys but not
    # what the binding answers. Nine, and the one that put it here was
    # MAX_PLAYER_LEVEL_TABLE[GetAccountExpansionLevel()] against a table holding
    # 0, 1 and 2 while the binding counted from one: on Wrath it read nothing,
    # MAX_PLAYER_LEVEL became nil, and the comparison beside it raised on every
    # level gained. Five of the nine call a C binding and all five have been
    # read; a tenth means someone should read that one. 1s.
    ("token_table_check.py",
     r"^(\d+) table lookup\(s\) keyed by a binding", 9,
     "tables the interface indexes with a binding's answer"),
    # A bound binding answering "" or 0 where the interface tests for nothing.
    # Only nil and false are false in Lua, so the branch meant for "there is
    # none" never runs and the one meant for "here it is" runs empty-handed:
    # GetAbandonQuestItems offered "you will lose:" with nothing after the
    # colon, and GetGuildBankTabCost kept the buy screen up for a guild that
    # owned every tab. Zero, and the first of those was put back in its original
    # shape and seen to trip it - the first draft could not, because it only
    # matched `if ( X() )` and both faults are written `local v = X()` and
    # tested on the next line. 2s.
    ("framexml_falsey_expected.py",
     r"^(\d+) answer something true where nothing was meant", 0,
     "bindings answering true where the interface tests for nothing"),
    # A binding spread into a vararg call, answering nothing. Nothing is
    # unpacked at these sites, so no short-return check reads them: the count
    # *is* the payload, and zero means the call runs its loop zero times and
    # the subsystem behind it does nothing. GetChatWindowMessages answered
    # nothing and that was the whole of chat - ChatFrame_OnLoad registers a
    # chat frame for no CHAT_MSG_ event at all, every one comes from the line
    # this feeds, so the window showed nothing from login to logout while every
    # message parsed correctly. Zero, and the stub was put back and seen to
    # trip it. 2s.
    ("framexml_vararg_spread.py",
     r"^(\d+) answer nothing at all", 0,
     "bindings spread into a vararg call that answer nothing"),
    # A C binding for a name FrameXML declares itself never runs: bindings are
    # registered in LuaEngine::initialize and the interface is read after, so
    # the later definition wins. Nothing else reports it - the binding compiles,
    # the name resolves, and every sweep that counts bound names counts it.
    # Usually the arrangement is deliberate and FrameXML's version wraps the
    # binding under another name; the three that do are listed in the tool with
    # what settled each. Zero, and dropping one from that list reports it. 2s.
    ("framexml_lua_override_check.py",
     r"^(\d+) C binding\(s\) doing real work that FrameXML overrides", 0,
     "C bindings FrameXML overrides, so they never run"),
    # GameTooltip setters that answer with a no-op, so the tooltip is blank.
    # A tooltip setter is the whole content of a tooltip: no error, no partial
    # result, just an empty box on one panel while every other hover works.
    # Eight on 2026-08-05, four fixed - SetTradeTargetItem (its own twin was
    # written and it was not), SetShapeshift (the stance bar, on screen the
    # whole time for five classes), SetMerchantCostItem, SetLFGDungeonReward.
    # The four left have nothing behind them to print; each is named in the
    # tool.
    ("tooltip_setter_check.py",
     r"^(\d+) answered by the no-op fallback", 0,
     "tooltip setters that leave the tooltip blank"),
    # The loud half of the same question, and the one that must stay zero: a
    # tooltip setter that is neither implemented nor allowlisted raises and
    # takes the OnEnter with it.
    ("tooltip_setter_check.py",
     r"^(\d+) neither implemented nor allowlisted", 0,
     "tooltip setters that raise"),
    ("chat_type_coverage_check.py",
     r"^(\d+) chat type\(s\) SendChatMessage does not map", 0,
     "chat types FrameXML can send that SendChatMessage does not map"),
    ("emote_coverage_check.py",
     r"^(\d+) emote token\(s\) DoEmote cannot answer", 0,
     "emote tokens FrameXML can send that DoEmote cannot answer"),
    ("client_command_bridge_check.py",
     r"^(\d+) client command\(s\) FrameXML's chat cannot reach", 0,
     "client slash commands FrameXML's chat cannot reach"),
    ("global_vs_method_check.py",
     r"^(\d+) global\(s\) called by FrameXML and bound only", 0,
     "globals FrameXML calls that exist only as a widget method"),
    # Two, and each is safe for its own reason - which is why they are listed
    # rather than filtered. GetWintergraspWaitTime is guarded inside the same
    # expression that compares it (`nextBattleTime and nextBattleTime > 60`,
    # and `and` short-circuits); GetMapDebugObjectInfo would raise on
    # `size > 1`, but the loop that calls it runs
    # `for i=1, GetNumMapDebugObjects()` and that answers zero, so it never
    # runs at all.
    #
    # It was two, then three when the search stopped reading only bindings
    # written as named functions - 750 of 1322 - and is two again now that it
    # reads which *return position* was compared. A binding that answers a
    # boolean first and numbers after is correct and common (GetLFGQueueStats
    # leads with hasData, then thirteen numbers), and knowing only that the
    # body pushes a boolean somewhere called five of those a fault.
    ("framexml_bool_vs_number.py",
     r"^(\d+) binding\(s\) answer a boolean or nil", 2,
     "bindings answering a boolean or nil where a number is compared"),
    # The other half of the same fault, and the half that raises rather than
    # quietly taking the wrong branch: FrameXML hands a widget's answer
    # straight to another binding, and luaL_optnumber treats a boolean as
    # neither absent nor convertible. GetChecked answered a boolean and
    # QueryAuctionItems read it as isUsable, so the auction browse raised
    # before sending and the search came back empty with nothing logged -
    # false is not nil, so an unticked box raised too. Zero, because there is
    # no benign version: the callee asked for a number.
    ("framexml_bool_vs_number.py",
     r"^(\d+) boolean answer\(s\) handed to a binding that reads a number", 0,
     "booleans passed where the receiving binding reads a number"),
    # The third face of the same fault, and the one that cost the most: a
    # binding answering a number where FrameXML uses the answer as a name.
    # GetGuildRosterInfo gave the class id where classFileName was wanted, so
    # `if ( classFileName ) then RAID_CLASS_COLORS[classFileName]` passed its
    # guard, found nothing, and read a field off nil inside GuildStatus_Update
    # - which took the whole guild roster down, but only once somebody was
    # online. GetWhoInfo had it too, from its own copy of the class table.
    #
    # Zero, because there is no benign version: the guard beside every one of
    # these sites is the caller saying it already handles the value being
    # absent, and an answer of the wrong kind walks straight past it.
    ("framexml_key_returns.py",
     r"^(\d+) binding return\(s\) of the wrong kind for the table they index", 0,
     "bindings answering the wrong kind of key for the table it indexes"),
    ("framexml_unaccounted_frames.py",
     r"^\d+ top-level frames, (\d+) unaccounted", 37,
     "FrameXML frames neither handed over nor suppressed"),
    # The blind spot both widget-method sweeps had: they count the no-op
    # allowlist as answered, which is right for "does the call raise here" and
    # wrong for a caller that reads what comes back. A no-op returns nil, and
    # nil in a comparison raises one line later, inside a function that looks
    # unrelated. GetFieldSize sat in the allowlist while its one caller
    # compared a byte count against it, so the guild event log came out blank
    # whenever it had events to show. Zero, because there is no such thing as
    # a deliberate one: if a caller reads the answer, the method is not a no-op.
    ("framexml_noop_returns.py",
     r"^(\d+) whose answer is used where nil raises", 0,
     "no-op widget methods whose nil answer reaches a comparison"),
    # dispatchSlashCommand stops at the first handler and reports success even
    # when that handler errors, so any command FrameXML defines wins whether it
    # works or not. Zero is the honest ceiling for a handler that can do nothing
    # at all; the tool's second list - a dead call beside a live one - is two
    # Battle.net branches that cannot run and is not guarded.
    ("framexml_slash_shadowing.py",
     r"^(\d+) client command\(s\) whose handler has no live call", 0,
     "slash commands FrameXML takes over with a handler that cannot act"),
    # The blind spot the other arity sweep has by design: it skips handlers
    # that unpack at the top, because one handler usually serves many events.
    # Where a handler serves exactly one, that unpack IS the signature.
    ("framexml_handler_arity.py",
     r"^(\d+) single-event handler\(s\) unpacking more at the top", 0,
     "single-event handlers unpacking more than the client fires"),
    # What the two arity sweeps structurally cannot see: the count being right
    # while the values are in the wrong places. That is what the spellcast
    # events did - two fired where two were read, the second one wrong.
    #
    # Its own blind spot has a name now: ITEM_PUSH fires (itemId, count) where
    # a bag id and an icon were meant, and FrameXML unpacks it into arg1/arg2,
    # so neither side offers a kind to compare. The bag-push animation has
    # never played. Written up in the tool; it needs one look in-world to
    # settle the slot numbering, not more reading.
    ("framexml_event_order.py",
     r"^(\d+) argument\(s\) in the wrong position", 0,
     "event arguments of the wrong kind for the position they are in"),
    # The same question as framexml_event_order, asked of bindings instead of
    # events: the count is right and the values are in the wrong slots.
    ("framexml_return_order.py",
     r"^(\d+) return value\(s\) in the wrong position", 0,
     "binding return values of the wrong kind for their position"),
    # A panel that polls its own keybinding from inside its own draw stops
    # answering that key the moment the draw is gated off - which is what
    # handing the element over does. Three were live on 2026-08-05: the talent
    # frame, the guild roster and the dungeon finder.
    ("keybinding_route_check.py",
     r"^(\d+) that would stop working when the panel is handed over", 0,
     "keys that stop working when their panel is handed over"),
    # The other half of the same seam: a window opened by writing its flag
    # rather than by calling a verb. Only the flags whose render is gated
    # matter - nine of thirty-two - which is what makes the list readable.
    ("window_flag_check.py",
     r"^(\d+) write\(s\) to one of those flags with no ownership check", 0,
     "window flags written without asking which interface owns the window"),
    # "Owned or suppressed" applied to dialogs one at a time. Seven were drawn
    # twice on 2026-08-05, three of them under the plain defaults. The shared
    # quest joined them later the same day, once QUEST_ACCEPT_CONFIRM started
    # being fired - which is exactly the day its reason stopped holding.
    #
    # Three left, each read: the duel countdown has no FrameXML counterpart at
    # all, the pet unlearn confirmation's CONFIRM_PET_UNLEARN exists here only
    # as a globalstring with no popup using it, and the battleground invite
    # needs CONFIRM_BATTLEFIELD_ENTRY, which nothing here fires. That last is a
    # thinner reason than the other two and stops holding the day it is wired.
    ("dialog_gate_check.py",
     r"^(\d+) with no ownership check", 2,
     "dialogs drawn without asking whether FrameXML draws them too"),
    ("api_shadowing_check.py",
     r"^\s*(\d+) to look at", 9,
     "names whose winner depends on load order"),
    # GameHandler forwards to the handlers it was split into, so its own copy
    # of a member is the fallback nothing takes. A writable accessor beside a
    # forwarding getter that hands out that copy gives a caller a list nobody
    # reads: the auction column sort reordered one and the mail sender backfill
    # filled in another, both to no visible effect and with nothing logged.
    # The second shape, and the one that got past the first arm: a member
    # handed out writable whose getter forwards, so the edit lands where
    # nothing reads. movement_handler cleared the gossip points of interest
    # that way and the markers stayed on the map.
    # The third shape: a writer that changes a member whose reader forwards.
    # resetDbcCaches cleared the talent and taxi caches on an expansion switch
    # while the getters beside them forwarded to the sub-handlers, so the
    # previous expansion's talents and flight points stayed live.
    ("forwarding_ref_check.py",
     r"^(\d+) writer\(s\) that change a member whose reader forwards", 0,
     "writers that change a member whose reader forwards"),
    ("forwarding_ref_check.py",
     r"^(\d+) member\(s\) edited through a reference nothing reads", 0,
     "members edited through a reference nothing reads"),
    ("forwarding_ref_check.py",
     r"^(\d+) member\(s\) written locally and read through a sub-handler", 0,
     "members written locally while every reader forwards"),
    # A function whose last caller moved away compiles, links and passes every
    # test. 102 members and six free functions had, across the GameHandler
    # decomposition and the FrameXML transition; what is left at two is a pair
    # the scan cannot see through, a call on a continuation line and a
    # multi-line qualified one. The .w* format headers are counted separately
    # because an unused accessor there is API rather than dead weight.
    ("dead_symbol_check.py",
     r"^(\d+) of those outside the \.w\* format headers", 2,
     "declared functions with no caller"),
    # `return 8` hands back the top eight of the stack, so a pop between the
    # values a binding built and its return slides the window down onto
    # whatever was underneath. GetAddOnInfo shipped that way: the entry table
    # arrived as the name and loadable as nil, so every addon read as
    # unloadable. Zero, and the tool is verified against the code as it stood
    # before the fix.
    ("lua_return_window_check.py",
     r"^(\d+) that pop after building them", 0,
     "bindings that pop after building their return values"),
    # One fact in two files is this codebase's commonest fault, and the four
    # pairs that remain are judged and named in the tool itself: two forwarding
    # facades, the classic and WotLK item queries, and the in-world versus
    # paper-doll geoset pick. Anything else is new.
    #
    # Only the cross-file number is pinned. The within-file count is reported
    # but not ratcheted, because it moves for reasons that are not regressions:
    # collapsing a four-line filter into a call brings previously separated
    # code within one twelve-line window, and the count goes up while the
    # duplication goes down. It did exactly that on the WMO queries.
    # Measured in code lines, so dense commenting is not penalised and removing
    # a comment cannot improve the number. The duplication sweeps above are at
    # zero, leaving function length as the remaining structural measure. The
    # largest entries are registration tables whose entries are inline lambdas.
    # The ceiling comes down as functions are split and must not go up.
    ("long_function_check.py",
     r"^(\d+) function\(s\) over \d+ code lines", 30,
     "functions too long to hold in one's head"),
    ("duplicate_block_check.py",
     r"^(\d+) file pair\(s\) sharing code", 0,
     "unjudged pairs of files sharing a block of code"),
]

# Prose rather than a count: the chunk checker says one of two sentences.
# Three more that were written, left outside this guard, and quietly kept
# working. Being outside it is not harmless - a sweep nobody runs finds its
# fault on the day someone happens to run it rather than the day it broke.
#
# Each of these three was canaried before it was pinned: the defect it claims
# to catch was written into a live interface file and each count went 0 -> 1.
# Two others were written up alongside them and are *not* here -
# framexml_nil_arithmetic's zero line and framexml_for_limit_check's sentence
# both stayed clean with their own fault sitting in a loaded file, so pinning
# them would have added two guards that cannot fail. (nil_arithmetic still
# earns its keep by hand: the line that found the calendar bug is its "carriers
# a live file reaches", which is 1 rather than 0 and so has no ceiling to pin.)
CHECKS += [
    ("framexml_contract_check.py",
     r"^(\d+) arity, \d+ return-count", 0,
     "bindings needing an argument the interface calls bare"),
    ("framexml_nil_use_check.py",
     r"^(\d+) missing functions used where nil raises", 0,
     "missing functions used where nil raises"),
    ("framexml_method_check.py",
     r"^(\d+) methods called that neither the metatable nor the known set", 0,
     "widget methods nothing answers at all"),
    # The one fault here that is not a wrong answer but a dead process. Lua
    # gives a binding twenty slots above its arguments and pushing past them
    # writes outside the stack - GetChildren on UIParent, with 267 of them,
    # died in realloc rather than raising. Canaried by removing each of the
    # three guards in turn; all three are reported.
    ("lua_stack_room_check.py",
     r"^(\d+) binding\(s\) push per row without asking for the room", 0,
     "bindings pushing a value per row with no room asked for"),
]

SENTENCES = [
    # The one check that opens the panels rather than reading about them. A
    # widget method that exists and answers nil is not a missing name, not a
    # short return and not a type mismatch, so it passes every static sweep
    # here and raises the first time a handler reaches through it - which is
    # what kept the calendar shut. Canaried: with the region GetParent binding
    # removed, this reports the calendar and nothing else.
    ("framexml_addon_open_check.py",
     "without raising, and none raises while the interface ticks",
     "a load-on-demand panel that raises on being opened or ticked"),
    ("bootstrap_chunk_check.py",
     "every local they use is declared in the chunk that uses it",
     "a bootstrap chunk using a local another chunk declared"),
    # The other one that builds rather than reads. A schema row can be right in
    # every static check and still draw a dropdown whose text belongs to a
    # different index - both halves correct, the mapping between them wrong,
    # which is what the login screen's parallax control did.
    #
    # It covers what a panel promises a player: a control shows the value it
    # holds, a dropdown shows the label its index names and writes that index
    # when chosen, moving a control writes the setting, Defaults puts every
    # setting back, Cancel leaves the panel as it was found while Okay keeps a
    # change against a Cancel after it, a quality preset moves what it covers
    # without asking for less than the one below, a value past the end of a row
    # is held to it, and a CVar a Blizzard control writes reaches its setting.
    #
    # Each was canaried against a fault this codebase had. The report names
    # which promise broke and for which setting.
    # What an unset CVar reads as, said twice: GetCVar answers the interface,
    # storedCVarValue answers the client where the setting is used. Both are
    # individually reasonable and both sides work; a difference only shows on
    # screen, as healing numbers over an unticked box. Canaried by moving one.
    # Settings kept in the config and acted on by nothing. A value written and
    # read back looks like a working setting from every angle: the key is there,
    # it survives a restart, the field holds what the file said. None of that
    # asks whether anything acts on it, and three fields from the chat window
    # this client no longer draws were kept that way for a long time.
    ("persisted_but_unread_check.py",
     "is read by something that uses it",
     "a setting kept in the config file that nothing acts on"),
    # The apply-once latches. A saved setting reaches its subsystem on the first
    # frame that subsystem exists, and a bool remembers that it has. Latching as
    # soon as the *outer* thing existed is how gamma came to be read from the
    # file and handed to nothing - the pipeline is built after the renderer, and
    # the latch only waited for the renderer. Canaried by hoisting one
    # assignment out of its guard.
    ("startup_latch_check.py",
     "is set inside the test for the thing it applies to",
     "a saved setting marked delivered on a frame where nothing took it"),
    # A setting bound, applied, saved and loaded, with nothing anywhere to
    # change it. Three are in that state on purpose and are named in the script.
    ("settings_without_a_control.py",
     "has something to change it with",
     "a setting reachable only by editing settings.cfg by hand"),
    # Feature detection against an interface whose missing names answer with a
    # truthy no-op. Escape shipped asking three close-functions this way: the
    # first answered whether or not it existed, and the press never reached the
    # branch that opens the game menu.
    ("interface_feature_check.py",
     "a missing one cannot answer",
     "a Lua global tested for truth, which a missing one passes"),
    # What this client draws itself, scaled to the screen. Four things pick a
    # default from the height and they have to pick the same, or neighbouring
    # parts of one HUD come up at different sizes - the buff bar was at 2.0 on a
    # 2160-line screen with everything beside it at 1.2. Canaried by putting one
    # back on the steps it used.
    ("settings_display_scale.py",
     "follows the screen the same way",
     "a default that does not scale with the screen the way the buff bar does"),
    # The file itself, moved value by value and read back. The restart check
    # drives settings through Lua, which cannot reach where the quest tracker
    # sits or how big the chat window is - and those are in the same file. The
    # count is pinned because the values come from the file being checked: a key
    # that stops being written is one fewer thing checked, not a failure.
    ("settings_file_round_trip.py",
     "is there when it is read back",
     "a value that does not survive being written to the settings file and read"),
    # The only check here that restarts. Everything else watches a setting
    # inside one run, and coming back is the whole of what a setting is for:
    # two runs over one config root, the first setting them and the second
    # reading its config from disk the way the client does. Canaried by taking
    # out the write-back to the CVar store, which brings all four back on their
    # defaults.
    ("settings_survive_restart.py",
     "still there in the next",
     "a setting that does not survive a restart"),
    # Blizzard's sliders against the room the settings behind them have. Two
    # handed over numbers their setting could not hold: Ground Density counts
    # doodads against a proportion, and Mouse Sensitivity is a multiplier
    # against an amount. Neither read as a slider doing nothing - each position
    # wrote a different number - so this follows the whole chain, the declared
    # range through the binding's scale to the loader's clamp. Canaried by
    # taking either scale back off, which names that slider and its numbers.
    ("cvar_slider_range_fit.py",
     "lands inside the range its setting holds",
     "a Blizzard slider handing over a number its setting has no room for"),
    # And the same defaults against the game's own tables, which declare one
    # beside each CVar their panels drive. Two differ on purpose and are named
    # in the script with the reason; this is for a third nobody decided on.
    ("cvar_default_vs_blizzard.py",
     "matches the game's own, but for",
     "a default differing from the game's own with no reason recorded"),
    ("cvar_default_agreement.py",
     "answers the interface the same way",
     "a CVar meaning one thing to the options panel and another to the client"),
    ("framexml_settings_control_check.py",
     "every control shows the value it is given",
     "a settings control not keeping one of the promises its panel makes"),
]


# The game data a sweep reads is not in the repository. Data/expansions and
# Data/opcodes are tracked; the extracted interface and the DBC files are the
# player's own, so CI has a Data directory with neither in it.
#
# A sweep whose input is absent does not report zero - it reports whatever it
# can see, which is nothing, and the shape of that report is not the shape this
# guard reads. Eleven of them came back either unreadable or wildly over
# ceiling on a checkout without the interface: framexml_promised_frames counted
# 226 frames "nothing declares" because nothing declares anything when there
# are no XML files to declare it in.
#
# Skipped and said so, which is what the framexml_run checks below already do.
# A guard that cannot see its subject must say so rather than pass or fail.
DATA_INPUTS = {
    "Data/interface": ROOT / "Data/interface",
    "Data/db": ROOT / "Data/db",
    # The server source these compare against is a separate local clone, so it
    # is absent everywhere except a developer machine that has one.
    "azerothcore": pathlib.Path("/home/k/azerothcore-wotlk"),
}


def missing_input(tool):
    """The input this sweep needs and this checkout does not have."""
    try:
        source = (TOOLS / tool).read_text()
    except OSError:
        return None
    for path, directory in DATA_INPUTS.items():
        if path in source and not directory.is_dir():
            return path
    # The headless runner is not part of the default build, so a sweep that
    # drives it has nothing to drive until someone asks for that target. The
    # checks further down already report this as a skip; a sweep in the tables
    # above went to the failure list instead.
    if "framexml_run" in source and not (ROOT / "build/bin/framexml_run").is_file():
        return "build/bin/framexml_run"
    return None


#: Sweeps that plant their own canaries and so need no population line: each
#: reintroduces the fault it looks for and fails if it is not reported, which
#: is a stronger statement than any count.
SELF_CANARYING = {"bounded_log_check.py", "copy_paste_axis_check.py"}


#: What each tool printed to stdout, kept apart from stderr for the population
#: rule below: a traceback carries line numbers, and counting those would let a
#: sweep that crashed satisfy a check meant to prove it looked at something.
# Where the runner keeps its config for these checks.
#
# It reads the CVar store at start-up, so a run inherits whatever the last one
# left - and the checks below drive it from several places, as do two of the
# scripts in TOOLS. Setting it here gives this guard's own runs one root; a
# script that wants its own overrides it in the environment it passes on. The
# alternative is a check whose result depends on which other check ran first.
os.environ.setdefault("WOWEE_CONFIG_ROOT", str(ROOT / "logs/sweep_guard_config"))

STDOUT = {}


def run(tool):
    out = subprocess.run([sys.executable, str(TOOLS / tool)],
                         capture_output=True, text=True)
    STDOUT[tool] = out.stdout
    return out.stdout + out.stderr


def check_rebuild_idiom():
    """`panel:Hide(); panel:Show()` must fire OnHide and then OnShow.

    That pair is how FrameXML asks a panel to rebuild itself, and it is the
    entire body of QuestFrame's handler for QUEST_DETAIL, QUEST_PROGRESS,
    QUEST_COMPLETE and QUEST_GREETING - the four NPC dialogs. Everything those
    panels display is positioned by QuestInfo_Display, which runs from OnShow
    and nowhere else.

    It is worth a guard of its own because of how it fails. Firing OnShow is
    noticed by comparing a frame against the last state anything reported for
    it, and a change that undoes itself before anything looks leaves nothing to
    compare - the panel opens, the buttons work, and the text keeps the place
    its XML gave it, outside the scroll frame that clips it. Nothing raises and
    nothing is logged.

    All three cases, because the middle one is only right if the outer two are:
    a plain Show() must fire, the pair must fire both, and a redundant Show()
    on something already shown must stay silent.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "the Hide();Show() rebuild idiom fires OnHide and OnShow"
    if not exe.exists() or not data.is_dir():
        return None, what
    setup = (
        "P = CreateFrame('Frame', 'SweepRebuildProbe', UIParent)\n"
        "P:SetPoint('TOPLEFT', UIParent, 'TOPLEFT', 10, -10)\n"
        "P:SetWidth(50) P:SetHeight(50)\n"
        "SHOWN = 0 HIDDEN = 0\n"
        "P:SetScript('OnShow', function() SHOWN = SHOWN + 1 end)\n"
        "P:SetScript('OnHide', function() HIDDEN = HIDDEN + 1 end)\n"
        "P:Hide()\n"
    )
    argv = [
        str(exe), str(data), setup,
        "--tick:1",
        "SHOWN = 0 HIDDEN = 0 P:Show()",
        "--tick:1",
        "if SHOWN ~= 1 then error('a plain Show() did not fire OnShow - this "
        "check cannot see OnShow at all, so its other answers mean nothing') "
        "end",
        "SHOWN = 0 HIDDEN = 0 P:Hide() P:Show()",
        "--tick:1",
        "if HIDDEN ~= 1 or SHOWN ~= 1 then error('Hide();Show() fired OnHide x'"
        "..HIDDEN..' OnShow x'..SHOWN..', wanted one of each - a panel asked to "
        "rebuild itself will not') end",
        "SHOWN = 0 P:Show()",
        "--tick:1",
        "if SHOWN ~= 0 then error('Show() on an already-shown frame fired "
        "OnShow, which re-runs handlers retail does not') end",
    ]
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return False, what + " (timed out)"
    if out.returncode == 0:
        return True, what
    # The runner echoes each expression back on its own `== ` line before
    # running it, so the text of a failing check appears in the output whether
    # or not it fired. Only the indented lines under it are what was raised -
    # matching on the message alone reported this as broken while it worked.
    detail = next((ln.strip() for ln in (out.stdout + out.stderr).splitlines()
                   if ln.startswith("   ") and "OnShow" in ln), "")
    return False, what + (" - " + detail if detail else "")


def check_removed_controls_are_gone():
    """Every control the client removes is gone, and still names a real frame.

    kRemovedControlsLua names about thirty frames and takes each off its panel.
    It is one Lua chunk, so one syntax error anywhere in it stops the whole list
    applying and every control it ever removed comes back - offering settings
    this client cannot honour, silently, with the game running and nothing
    raised. That happened once, from a comment written with // instead of --.

    Names go stale the same way. A frame renamed or misspelled removes nothing,
    and the list keeps claiming it. Nine entries once named CVars rather than
    the controls in front of them, and four more named a nesting that does not
    exist, all reading as a tidy list of handled settings.

    The first thing checked is a control that must NOT be removed. Without it a
    probe that cannot see visibility at all would report a clean list.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "the controls this client removes are gone, and still name frames"
    if not exe.exists() or not data.is_dir():
        return None, what

    header = (ROOT / "include/addons/addon_lua_snippets.hpp").read_text(errors="ignore")
    start = header.find("kRemovedControlsLua")
    end = header.find(')LUA', start) if start != -1 else -1
    if start == -1 or end == -1:
        return False, what + " - kRemovedControlsLua not found"
    body = header[start:end]
    # The control list only: the category list below it is checked separately.
    listing_src = body[:body.find("kRemovedCategories")] if "kRemovedCategories" in body else body
    names = re.findall(r'^\s*"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?$', listing_src, re.M)
    cats = re.findall(r'^\s*"([A-Za-z0-9_]+Panel)",\s*(?:\.\w+\s*=\s*)?$',
                      body[body.find("kRemovedCategories"):], re.M) if "kRemovedCategories" in body else []
    if not names:
        return False, what + " - no removed control names parsed"

    # A control deliberately left alone, so this probe has something that must
    # still be visible. If it ever gets removed, pick another.
    live = "InterfaceOptionsCombatPanelEnemyCastBarsOnNameplates"
    listing = ", ".join(f'"{n}"' for n in names)
    catlist = ", ".join(f'"{c}"' for c in cats)
    probe = (
        f"local live = _G['{live}']\n"
        "if not live or type(live.IsShown) ~= 'function' or not live:IsShown() then\n"
        f"  error('the control that must stay ({live}) is not shown, so this "
        "probe cannot tell a removed control from a kept one') end\n"
        f"local names = {{{listing}}}\n"
        f"local cats = {{{catlist}}}\n"
        "local bad = {}\n"
        "for _, n in ipairs(names) do\n"
        "  local f = _G[n]\n"
        "  if not f or type(f.GetName) ~= 'function' or f:GetName() ~= n then\n"
        "    bad[#bad+1] = n .. ' (names no frame)'\n"
        "  elseif f:IsShown() then\n"
        "    bad[#bad+1] = n .. ' (still shown)'\n"
        "  end\n"
        "end\n"
        # A page taken out of the list must not be offered by it either.
        "for _, c in ipairs(cats) do\n"
        "  local p = _G[c]\n"
        "  if not p then bad[#bad+1] = c .. ' (names no panel)'\n"
        "  elseif not p.hidden then bad[#bad+1] = c .. ' (still listed)' end\n"
        "end\n"
        "if #bad > 0 then\n"
        "  local shown = {}\n"
        "  for i = 1, math.min(4, #bad) do shown[i] = bad[i] end\n"
        "  local tail = (#bad > #shown) and (' and ' .. (#bad - #shown) .. ' more') or ''\n"
        "  error(#bad .. ' removed in name only: ' .. table.concat(shown, ', ') .. tail)\n"
        "end\n"
    )
    argv = [str(exe), str(data), "--lua:" + probe]
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return False, what + " (timed out)"
    if out.returncode == 0:
        return True, what + f" ({len(names)} controls, {len(cats)} pages)"
    detail = next((ln.strip() for ln in (out.stdout + out.stderr).splitlines()
                   if ln.startswith("   ") and ("removed in name only" in ln
                                                or "must stay" in ln)), "")
    return False, what + (" - " + detail if detail else "")


def check_paragraph_wrapping():
    """A font string with a declared width and no height wraps inside it.

    `<AbsDimension x="285" y="0"/>` is WoW's wrapping paragraph and 240 font
    strings across the interface declare one - every quest description and
    objective, every mail body, every gossip greeting. The zero is not a
    missing height, it is "however tall the wrap makes me".

    Read as a request to be measured instead, the string keeps the width of
    whatever sentence it happens to hold and one line's height: it draws out
    through the side of the frame that clips it, and whatever anchors below it
    sits on top of the lines that should have pushed it down. Nothing raises.

    Both halves, because either alone can pass while the other is broken: the
    declared width has to survive, and the height has to follow the text.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "a font string with a declared width wraps inside it"
    if not exe.exists() or not data.is_dir():
        return None, what
    # QuestInfoDescriptionText is the reported one and declares x=285 y=0.
    argv = [
        str(exe), str(data),
        "T = QuestInfoDescriptionText T:SetText('Short.')",
        "--tick:1",
        "W1 = T:GetWidth() H1 = T:GetHeight()",
        "T:SetText('A much longer line of prose that has to wrap onto at least "
        "three separate lines inside the two hundred and eighty five pixels "
        "its XML gave it.')",
        "--tick:1",
        "if T:GetWidth() ~= 285 or W1 ~= 285 then error('the declared width did "
        "not survive: one line '..W1..', many lines '..T:GetWidth()..', wanted "
        "285 for both - the string was sized from its text instead of wrapped "
        "inside its box') end",
        "if T:GetHeight() <= H1 then error('height did not grow with the text: "
        "one line '..H1..', many lines '..T:GetHeight()..' - everything "
        "anchored below this sits on top of it') end",
    ]
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return False, what + " (timed out)"
    if out.returncode == 0:
        return True, what
    detail = next((ln.strip() for ln in (out.stdout + out.stderr).splitlines()
                   if ln.startswith("   ") and ("width" in ln or "height" in ln)),
                  "")
    return False, what + (" - " + detail if detail else "")


def check_binding_dispatch():
    """A key press reaches FrameXML's bindings, but not one the client answers.

    The interface ships 273 binding scripts and, until the dispatch existed, no
    press could reach any of them - a key bound in its own key-binding panel was
    recorded and then never honoured.

    The guard is really on the exclusion. A key answered by both the client and
    a binding is answered twice, and most bindings toggle something: twice means
    the panel opens and shuts on the one press, so the key reads as dead. That
    is worse than the gap it replaced and much harder to spot, because it looks
    exactly like the binding never ran.

    All three, since each alone can pass while the others are broken: a
    client-owned command must be declined, an unbound key must do nothing, and a
    command the client has no path for must actually run.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "key presses reach bindings, except ones the client answers"
    if not exe.exists() or not data.is_dir():
        return None, what
    argv = [str(exe), str(data),
            "--bind:W",                                   # MOVEFORWARD, polled
            "--bind:B",                                   # TOGGLEBACKPACK, live
            "--bind:X",                                   # nothing bound
            "SetBinding('J', 'OPENALLBAGS')", "--bind:J"] # no client path
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return False, what + " (timed out)"
    text = out.stdout + out.stderr
    wanted = [
        ("W -> declined MOVEFORWARD",
         "movement is polled every frame; a binding for it would move twice"),
        ("B -> declined TOGGLEBACKPACK",
         "the client opens the bags itself; a binding would shut them again"),
        ("X -> nothing bound to this key",
         "an unbound key must stay silent"),
        ("J -> ran OPENALLBAGS",
         "a command the client has no path for must actually run"),
    ]
    for needle, why in wanted:
        if needle not in text:
            return False, f"{what} - expected '{needle}' ({why})"
    return True, what


CLICK_EVERYTHING = (
    # Everything visible and mouse-enabled, clicked once.
    #
    # Visible only: clicking a button in a panel nobody opened raises for
    # reasons that are not faults. Mouse-enabled only: Click() bypasses that
    # check itself, so a sweep that skips it presses buttons the cursor could
    # never reach.
    #
    # The pcall here is not what reports a raise, and reading it is a mistake
    # that cost one clean-looking sweep across eight panels: a raise inside a
    # frame script is caught by the engine and handed to its error callback, so
    # pcall never sees it. The runner's own error report is the count that
    # means anything. This pcall only stops one dead button ending the sweep.
    "local names = {} "
    "for k, v in pairs(_G) do "
    "  if type(k) == 'string' and type(v) == 'table' and v.Click "
    "     and v.IsVisible and v.IsMouseEnabled and v.GetObjectType then "
    "    local ok, t = pcall(function() return v:GetObjectType() end) "
    "    if ok and (t == 'Button' or t == 'CheckButton') then "
    "      local vis, me = false, false "
    "      pcall(function() vis = v:IsVisible() me = v:IsMouseEnabled() end) "
    "      if vis and me then names[#names+1] = k end "
    "    end "
    "  end "
    "end "
    "table.sort(names) "
    # Hovered as well as clicked. A tooltip is built in OnEnter and nowhere
    # else, and the mail tooltip died there on a colour argument the interface
    # passes as an empty string - five hovers later the OnUpdate driving it was
    # unhooked for the session, and nothing said so on screen.
    "for _, n in ipairs(names) do "
    "  local b = _G[n] "
    "  if b then "
    "    if b.GetScript then "
    "      local enter = b:GetScript('OnEnter') "
    "      if enter then pcall(function() enter(b) end) end "
    "    end "
    "    if b.Click then pcall(function() b:Click() end) end "
    "    if b.GetScript then "
    "      local leave = b:GetScript('OnLeave') "
    "      if leave then pcall(function() leave(b) end) end "
    "    end "
    "  end "
    "end")


NPC_DIALOG_DATA = """
local LONG = "Kobolds have overrun the mine to the east and the miners are afraid to return. Clear them out and bring me the candles they carry as proof of your work."
GetTitleText     = function() return "A Test Quest" end
GetQuestText     = function() return LONG end
GetObjectiveText = function() return "Kill 10 Kobold Vermin." end
GetProgressText  = function() return "Have you dealt with the kobolds yet? They grow bolder each day." end
GetRewardText    = function() return "You have done well. Take this as a token of the town's gratitude." end
GetGreetingText  = function() return "Welcome, traveller. I have work that needs doing, if you are willing." end
GetNumAvailableQuests = function() return 1 end
GetNumActiveQuests    = function() return 1 end
GetAvailableTitle     = function() return "A Test Quest" end
GetActiveTitle        = function() return "An Older Task" end
GetAvailableQuestInfo = function() return false, 0, 0, false end
IsQuestCompletable    = function() return true end
GetNumQuestItems      = function() return 0 end
GetNumQuestRewards    = function() return 0 end
GetNumQuestChoices    = function() return 0 end
GetNumQuestSpellRewards = function() return 0 end
GetQuestMoneyToGet    = function() return 0 end
GetRewardMoney        = function() return 0 end
GetRewardXP           = function() return 0 end
GetRewardHonor        = function() return 0 end
GetRewardArenaPoints  = function() return 0 end
GetRewardTalents      = function() return 0 end
GetRewardSpell        = function() return nil end
GetSuggestedGroupSize = function() return 0 end
QuestGetAutoAccept    = function() return false end
"""


def check_bags_tile():
    """Bags reopened in one breath are still tiled, not stacked.

    ContainerFrame_GenerateFrame writes `bags[bagsShown + 1]`, anchors the
    whole list, and only then calls Show - and `bagsShown` is maintained by
    ContainerFrame_OnShow and ContainerFrame_OnHide. OpenAllBags hides every
    open bag and reopens it in one breath, so if either handler is late the
    count is stale for the entire sequence: three reopened bags write
    themselves over one index and the anchor pass sees a list with a name
    missing. Every bag but the first then keeps the position its XML gave it,
    which is the same position - and walking up to a vendor with bags already
    open stacked them on top of each other.

    Counts the list rather than reading positions, because the list is what
    the anchor pass walks and a wrong list is the fault. Three bags in, three
    names out, and each anchored to a different frame.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "bags reopened together are tiled rather than stacked"
    if not exe.exists() or not data.is_dir():
        return None, what
    # The command the client actually sends, read out of the call site rather
    # than written again here - so putting OpenAllBags back would be run by
    # this check, and this check would fail.
    source = (ROOT / "src" / "ui" / "game_screen.cpp").read_text(errors="ignore")
    opener = re.search(r'kOpenBagsCommand\s*=\s*\n?\s*"([^"]+)"', source)
    if not opener:
        return False, what + " - kOpenBagsCommand is gone, so what the vendor " \
                             "sends is no longer what this checks"
    # Three chunks, not one. The fault needs a frame boundary between the bag
    # being opened and the vendor opening - which is the real situation, and
    # which is also when the deferred OnHide would have run. In a single chunk
    # both commands look identical and this check proves nothing.
    argv = [str(exe), str(data),
            "GetContainerNumSlots = function() return 16 end OpenBag(1)",
            opener.group(1),
            "local n = 0 "
            "for _ in ipairs(ContainerFrame1.bags) do n = n + 1 end "
            "local shown = 0 "
            "for i = 1, 13 do local f = _G['ContainerFrame'..i] "
            "  if f and f:IsShown() then shown = shown + 1 end end "
            "if n ~= shown then error('the bag list holds '..n..' names for '"
            "..shown..' open bags - the anchor pass walks that list, so the "
            "ones missing from it keep the position their XML gave them, which "
            "is the same position') end "
            # A stack is the same anchor *and* the same offsets. Anchoring two
            # bags to UIParent is not one: that is how a second column starts,
            # and those two differ by a column's width.
            "local seen = {} "
            "for _, name in ipairs(ContainerFrame1.bags) do "
            "  local point, rel, relPoint, x, y = _G[name]:GetPoint(1) "
            "  local key = tostring(point)..'|'..tostring(rel and rel:GetName() or '?') "
            "    ..'|'..tostring(relPoint)..'|'..tostring(x)..'|'..tostring(y) "
            "  if seen[key] then error('two bags share an anchor point exactly "
            "- they are drawn in the same place') end "
            "  seen[key] = true "
            "end"]
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return False, what + " (timed out)"
    if out.returncode != 0:
        # The runner echoes the script it was given before running it, and
        # that echo contains the error text too; the line that matters is the
        # one Lua raised, which carries its chunk name.
        detail = next((ln.split(": ", 2)[-1].strip()
                       for ln in (out.stdout + out.stderr).splitlines()
                       if "script error" in ln), "")
        return False, what + (" - " + detail if detail else "")
    return True, what


def check_npc_dialogs_fill():
    """All four NPC dialogs put their text on screen.

    This is the reported bug - a parchment with working buttons and no text -
    and it had two independent causes, either of which alone reproduces it:
    the panel never ran its OnShow, so nothing was ever positioned; and a
    paragraph sized itself from its text instead of wrapping, so it ran out
    through the side of the frame that clips it.

    All four panels, because all four are the same idiom and only the detail
    one was ever measured. The vendor is deliberately absent: it was the one
    dialog that always worked, and it works by a different route.

    Three things per panel, since each can hold while the others fail: the
    string has text, it has height (a paragraph laid on one line reports the
    height of one line however much it holds), and it is visible.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "all four NPC dialogs fill themselves in"
    if not exe.exists() or not data.is_dir():
        return None, what
    # Each panel with the binding that feeds it, because the event is fired
    # twice and what the second one must show is new text.
    #
    # Twice is the whole point. Opening a panel for the first time makes it
    # visible, and that is a change anything watching can see - so the first
    # fire works even with the rebuild broken. The second arrives with the
    # panel already open, which is the case FrameXML uses Hide();Show() for and
    # the only one that fails. A single fire passed this check with the fix
    # removed, which is how the shape of it was found.
    panels = [
        ("QUEST_GREETING", "GreetingText",             "GetGreetingText"),
        ("QUEST_DETAIL",   "QuestInfoDescriptionText", "GetQuestText"),
        ("QUEST_PROGRESS", "QuestProgressText",        "GetProgressText"),
        ("QUEST_COMPLETE", "QuestInfoRewardText",      "GetRewardText"),
    ]
    second = ("The second time this dialog opens it must say something else "
              "entirely, and it must wrap onto more than one line to say it.")
    for event, element, getter in panels:
        argv = [str(exe), str(data), NPC_DIALOG_DATA,
                "--fire:" + event, "--tick:2",
                f"local f = {element} "
                f"if not f or f == _G['QQNoSuchThing'] then "
                f"error('{event}: {element} does not exist') end "
                f"if not f:GetText() or f:GetText() == '' then "
                f"error('{event}: {element} has no text - the panel never "
                f"filled itself in') end",
                f"{getter} = function() return [[{second}]] end",
                "--fire:" + event, "--tick:2",
                f"local f = {element} "
                f"if f:GetText() ~= [[{second}]] then "
                f"error('{event}: {element} kept its first text when the "
                f"dialog opened again - the panel did not rebuild') end "
                f"if f:GetHeight() <= 0 then "
                f"error('{event}: {element} has no height') end "
                # Two lines at least. The text above is far too long to fit on
                # one at any of these widths, so a single line's height means it
                # was sized from its own text rather than wrapped inside its
                # box - which is the other cause of the same blank dialog, and
                # the one a height-above-zero test sails straight past.
                f"if f:GetHeight() < 25 then "
                f"error('{event}: {element} is one line tall holding text that "
                f"cannot fit on one - it was sized from its text instead of "
                f"wrapped inside its width') end "
                f"if not f:IsVisible() then "
                f"error('{event}: {element} is not visible') end"]
        try:
            out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
        except subprocess.TimeoutExpired:
            return False, f"{what} ({event} timed out)"
        if out.returncode != 0:
            detail = next((ln.strip() for ln in
                           (out.stdout + out.stderr).splitlines()
                           if ln.startswith("   ") and event in ln), event)
            return False, what + " - " + detail
    return True, what


def check_nothing_unsized():
    """Nothing on a panel is on screen with no room to draw in.

    A region with no width or no height is not drawn, and nothing about it reads
    wrong from Lua: it is shown, it has its texture, every property answers
    correctly, and only the one number that decides whether any of it reaches
    the screen is missing. Sixteen tab highlights and a status icon were in that
    state.

    These three panels between them cover both ways it happened - art whose
    anchors size one axis and leave the other open, and a texture with one
    anchor and no size at all, which takes the dimensions of its own image.
    That last needs the assets, so this reports a skip rather than a pass when
    they are not there: a clean answer from a run that could not read a single
    texture would mean nothing.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "nothing on a panel is drawn with no room to draw in"
    if not exe.exists() or not data.is_dir():
        return None, what
    for panel in ("CharacterFrame", "FriendsFrame", "MerchantFrame"):
        argv = [str(exe), str(data),
                f"local f = {panel} if f and f ~= _G['QQNoSuchThing'] then "
                f"ShowUIPanel(f) end",
                "--tick:3", "--unsized"]
        try:
            out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
        except subprocess.TimeoutExpired:
            return False, f"{what} ({panel} timed out)"
        text = out.stdout + out.stderr
        if "texture sizes are unavailable" in text:
            return None, what + " (no assets)"
        m = re.search(r"^\s+(\d+) unsized, of (\d+) carrying", text, re.M)
        if not m:
            return False, f"{what} - {panel} reported no count at all"
        # The second number matters as much as the first: zero unsized is also
        # what "nothing on screen has anything to show" looks like.
        if int(m.group(2)) < 50:
            return False, (f"{what} - {panel} had only {m.group(2)} regions "
                           f"carrying anything, so a zero here proves nothing")
        if int(m.group(1)) != 0:
            names = [ln.strip() for ln in text.splitlines()
                     if ln.strip().startswith("unsized:")]
            return False, (f"{what} - {panel}: " +
                           "; ".join(names[:4]))
    return True, what


def check_panels_without_the_standin():
    """Open panels with a player, no stand-in, and draw them.

    Three things that had only ever been done separately. The stand-in makes an
    unknown global answer with a no-op, so anything leaning on one stays quiet;
    a player makes the panels that read UnitClass work at all; and drawing is
    where a whole class of fault lives and was unreachable until the harness
    could paint into a draw list.

    Together they found SetAuctionsTabShowing raising on the boolean the
    interface actually passes it - from AuctionFrame's OnShow, so the auction
    house raised the moment it opened, and again on every tab click. That is
    not a stand-in fault and was not hidden by one: it raised in ordinary runs
    too, and nothing had ever opened the panel to see it.

    A raise inside a handler is swallowed by the client, so this is the only
    way to see one. Skipped rather than passed without assets.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "panels open, click, draw and raise nothing with no stand-in"
    if not exe.exists() or not data.is_dir():
        return None, what
    env = dict(os.environ, WOWEE_LUA_API_FALLBACK="0")
    for panel in ("AuctionFrame", "CharacterFrame", "MerchantFrame",
                  "SpellBookFrame", "QuestLogFrame"):
        # Clicked as well as opened. A handler only runs when something runs
        # it, and clicking through a panel is what loads the calendar - which
        # is where CalendarCanSendInvite was found missing.
        argv = [str(exe), str(data), "--player",
                f"local f = {panel} if f and f ~= _G['QQNoSuchThing'] then "
                f"ShowUIPanel(f) end",
                "--tick:3", CLICK_EVERYTHING, "--tick:2", "--draw"]
        try:
            out = subprocess.run(argv, capture_output=True, text=True,
                                 timeout=300, env=env)
        except subprocess.TimeoutExpired:
            return False, f"{what} ({panel} timed out)"
        raised = [ln.strip() for ln in (out.stdout + out.stderr).splitlines()
                  if re.match(r"^\s+\S.*:\d+:", ln)]
        if raised:
            return False, f"{what} - {panel}: {raised[0][:150]}"
    return True, what


def check_dialogs_without_the_standin():
    """The event-driven dialogs open, take a click and raise nothing.

    Panels are reached by opening them; dialogs are reached by an event, and
    they are where the reported faults keep landing. Same three conditions as
    the panel sweep - a player, no stand-in, a real draw - plus a click on
    everything visible, because a handler only runs when something runs it.

    PETITION_SHOW is deliberately absent. Fired with no petition stored,
    GetPetitionInfo answers nothing and PetitionFrame_Update raises on
    `for i=1, minSignatures`; but this client fires that event only after
    parsing and storing the charter, from both of the two places that send it.
    A harness that reaches a state the client cannot is worse than one that
    reaches less, and a permanent failure here would drown the real ones.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "the event-driven dialogs open, click and raise nothing"
    if not exe.exists() or not data.is_dir():
        return None, what
    env = dict(os.environ, WOWEE_LUA_API_FALLBACK="0")
    for event in ("QUEST_GREETING", "QUEST_DETAIL", "QUEST_PROGRESS",
                  "QUEST_COMPLETE", "GOSSIP_SHOW", "MERCHANT_SHOW",
                  "LOOT_OPENED", "TRADE_SHOW", "TRAINER_SHOW"):
        argv = [str(exe), str(data), "--player", NPC_DIALOG_DATA,
                "--fire:" + event, "--tick:3", CLICK_EVERYTHING, "--tick:2",
                "--draw"]
        try:
            out = subprocess.run(argv, capture_output=True, text=True,
                                 timeout=300, env=env)
        except subprocess.TimeoutExpired:
            return False, f"{what} ({event} timed out)"
        raised = [ln.strip() for ln in (out.stdout + out.stderr).splitlines()
                  if re.match(r"^\s+\S.*:\d+:", ln)]
        if raised:
            return False, f"{what} - {event}: {raised[0][:150]}"
    return True, what


def check_tooltip_colour_arguments():
    """A tooltip colour that is not a number is a default, not an error.

    Blizzard writes `AddLine(ENCLOSED_MONEY, "", 1, 1, 1)` - an empty string
    where the red goes - at seven places: the mail's money and COD lines, the
    bag and paperdoll repair costs, the taxi map's "you are here", the pet
    action bar, and uiparent. luaL_optnumber takes a string it can convert and
    "" is not one, so every one of those raised.

    What that costs is out of all proportion to the argument: a raise inside
    OnEnter loses the whole tooltip, and the OnUpdate driving it is unhooked
    after five failures - so the tooltip dies for the session, silently.

    Not covered by the panel sweep, and worth saying why rather than assuming
    it is: MailItem1Button's OnEnter only reaches that line when the mail
    carries money, so with no mail behind it the hover walks straight past.
    Checked directly instead.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    what = "a tooltip colour that is not a number is taken as a default"
    if not exe.exists() or not data.is_dir():
        return None, what
    argv = [str(exe), str(data),
            # The exact shapes the interface uses.
            'GameTooltip:AddLine("Enclosed Money", "", 1, 1, 1)',
            'GameTooltip:AddDoubleLine("left", "right", "", 1, 1, "", 1, 1)',
            # And numbers still mean what they say, so this cannot pass by
            # ignoring the colours altogether.
            'GameTooltip:AddLine("coloured", 1, 0, 0)',
            'GameTooltip:AddDoubleLine("a", "b", 1, 0, 0, 0, 1, 0)']
    try:
        out = subprocess.run(argv, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return False, what + " (timed out)"
    if out.returncode == 0:
        return True, what
    detail = next((ln.strip() for ln in (out.stdout + out.stderr).splitlines()
                   if "AddLine" in ln and ":" in ln), "")
    return False, what + (" - " + detail[:150] if detail else "")


def check_without_the_standin():
    """Load the whole interface with the missing-API stand-in turned off.

    By default an unknown global answers with a stand-in rather than nil, so a
    file survives past a name nothing implements - and nothing says it was
    needed. With the stand-in off, anything the interface actually depends on
    raises and names itself.

    That property was earned one addon at a time and is lost silently: a new
    reference to something the client is supposed to create looks perfectly
    fine on the default run. This is the only check here that runs the
    interface rather than reading it, so it is also the only one that can see
    that.

    Skipped, not failed, when the runner has not been built -
    WOWEE_BUILD_FRAMEXML_RUN is off by default and most builds will not have
    it. A skip prints as a skip so it is never mistaken for a pass.
    """
    exe = ROOT / "build" / "bin" / "framexml_run"
    data = ROOT / "Data"
    if not exe.exists() or not data.is_dir():
        return None, "the interface loads with no missing-API stand-in"
    env = dict(os.environ, WOWEE_LUA_API_FALLBACK="0")
    try:
        out = subprocess.run([str(exe), str(data)], capture_output=True,
                             text=True, timeout=300, env=env)
    except subprocess.TimeoutExpired:
        return False, "the interface loads with no missing-API stand-in (timed out)"
    text = out.stdout + out.stderr
    load = re.search(r"^== load: (\d+) error", text, re.M)
    addons = re.search(r"^== addons: (\d+) of \d+ load-on-demand failed", text, re.M)
    login = re.search(r"^== login events: (\d+) error", text, re.M)
    if not (load and addons and login):
        return False, ("the interface loads with no missing-API stand-in "
                       "(could not read the run's own report)")
    bad = int(load.group(1)) + int(addons.group(1)) + int(login.group(1))
    return bad == 0, ("the interface loads with no missing-API stand-in "
                      f"({load.group(1)} load, {addons.group(1)} addon, "
                      f"{login.group(1)} login)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true",
                    help="print the ceilings and exit")
    args = ap.parse_args()

    if args.list:
        for tool, _, ceiling, what in CHECKS:
            print(f"  {ceiling:>3}  {what}   [{tool}]")
        for tool, _, what in SENTENCES:
            print(f"    -  {what}   [{tool}]")
        return 0

    # One run per tool, shared by its checks.
    outputs = {}
    for tool, *_ in CHECKS:
        outputs.setdefault(tool, None)
    for tool, *_ in SENTENCES:
        outputs.setdefault(tool, None)
    skipped = {}
    for tool in outputs:
        absent = missing_input(tool)
        if absent:
            skipped[tool] = absent
            continue
        outputs[tool] = run(tool)

    failures = []
    for tool, pattern, ceiling, what in CHECKS:
        if tool in skipped:
            print(f"  skip    -       {what} (no {skipped[tool]})")
            continue
        m = re.search(pattern, outputs[tool], re.M)
        if not m:
            failures.append(f"{tool}: could not read its own count for "
                            f"'{what}' - the report's shape changed, which "
                            f"makes this guard silently useless")
            continue
        found = int(m.group(1))
        status = "ok " if found <= ceiling else "OVER"
        print(f"  {status}  {found:>3} / {ceiling:<3}  {what}")
        if found > ceiling:
            failures.append(f"{tool}: {found} {what}, ceiling is {ceiling}")

    for tool, sentence, what in SENTENCES:
        if tool in skipped:
            print(f"  skip    -       {what} (no {skipped[tool]})")
            continue
        clean = sentence in outputs[tool]
        print(f"  {'ok ' if clean else 'OVER'}    -       {what}")
        if not clean:
            failures.append(f"{tool}: {what}")

    # Every sweep must say what it looked at, not only what it found.
    #
    # A sweep pinned at zero whose whole output is the digit zero cannot be
    # told apart from one whose matcher has stopped recognising its subject:
    # both print "0 ...", and this reads both as a pass. That is not
    # hypothetical - posix_only_check could not see std::localtime for as long
    # as it existed, and dead_symbol_check counted a name in a comment as a
    # call. Neither showed up here.
    #
    # The rule is only that a positive number appears somewhere in the output.
    # It costs nothing - these runs are already cached above - and it makes a
    # new sweep say how much it examined, which is the number that goes to zero
    # when the sweep goes blind.
    blind = []
    for tool in sorted(outputs):
        if tool in skipped or tool in SELF_CANARYING:
            continue
        if not any(int(n) > 0 for n in re.findall(r"\b(\d+)\b", STDOUT.get(tool, ""))):
            blind.append(tool)
    print(f"  {'ok ' if not blind else 'OVER'}  {len(blind):>3} / 0    "
          f"sweeps reporting nothing they looked at")
    for tool in blind:
        failures.append(f"{tool}: reports no population, so a matcher that has "
                        f"gone blind reads exactly like a clean tree")

    for ok, what in (check_without_the_standin(), check_rebuild_idiom(),
                     check_paragraph_wrapping(), check_binding_dispatch(),
                     check_npc_dialogs_fill(), check_bags_tile(),
                     check_nothing_unsized(),
                     check_panels_without_the_standin(),
                     check_dialogs_without_the_standin(),
                     check_removed_controls_are_gone(),
                     check_tooltip_colour_arguments()):
        if ok is None:
            print(f"  skip    -       {what} (framexml_run not built)")
            continue
        print(f"  {'ok ' if ok else 'OVER'}    -       {what}")
        if not ok:
            failures.append(f"framexml_run: {what}")

    if failures:
        print(f"\n{len(failures)} sweep(s) worse than the pinned ceiling:\n")
        for f in failures:
            print(f"  {f}")
        print("\nEach of these is a fault that raises nothing and fails no "
              "other test.\nFix it, or move the ceiling deliberately and say "
              "why in the commit.")
        return 1

    print("\nEvery sweep at or under its ceiling.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
