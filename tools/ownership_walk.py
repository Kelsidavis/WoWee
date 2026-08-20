"""Whether a line sits in an if/else chain that tests who owns an element.

Written for the three checks that asked it - window_route_check,
window_flag_check and dialog_gate_check - because all three had the same bug
independently and fixing it three times is how the three would drift apart.
The first two have been retired with the windows they watched; dialog_gate_check
is the one left asking.

THE BUG THIS EXISTS TO PREVENT

Each of those checks originally read a window of lines around the call and asked
whether `frameXmlOwns` appeared anywhere in it. That is wrong for the subject
they all have: runs of near-identical lines. Eleven micro-menu buttons in a row,
fourteen dialogs in a row, a block of slash-command branches. The line above an
ungated entry is almost always the *previous* entry's ownership check, so the
ungated one borrows it and the report comes out clean.

It was found three times, at three different window widths - twenty-four lines,
six, and two - each tuned narrower after the last was caught, and each still
wrong for the same reason. Proximity is not the question. The shape of the
branch is.

WHAT IT DOES

Answers the same line first, then walks upwards through exactly the shapes a
branch is made of - `else`, `else if`, a statement belonging to the other
branch, a comment - and stops at the first `if`, answering whether that one
tests ownership. For an entry with no branch of its own the first `if` reached
is the control itself (`if (button(...))`, `if (cmds.showWho)`), so a
neighbour's gate cannot be borrowed however close it sits.

Comments do not count against the bound. These branches carry long ones, and a
bound that counts them stops short of the gate being explained.
"""
import re

#: The literal call and the helpers named for the same question.
OWNERSHIP = ("frameXmlOwns", "AreFrameXml", "IsFrameXml", "frameXmlChat")

#: How many statements up to look before giving up. Generous, because the walk
#: stops at the first `if` regardless - the bound is a backstop, not the rule.
BUDGET = 12


def _mentions(line, words):
    return any(word in line for word in words)


def gated(lines, index, words=OWNERSHIP):
    """Is lines[index] inside a branch whose condition tests ownership?"""
    line = lines[index]
    if _mentions(line, words):
        return True

    # Whether an `else` has been passed on the way up. It decides what an
    # `if (C) stmt;` one-liner means: with an else behind us that line is our
    # own branch's condition, and without one it is the *previous entry* - a
    # whole dialog or button of its own, whose gate is not ours to borrow.
    seen_else = bool(re.match(r"\}?\s*else\b", line.strip()))

    budget = BUDGET
    for j in range(index - 1, -1, -1):
        text = lines[j].strip()
        if not text or text.startswith(("//", "/*", "*")):
            continue
        budget -= 1
        if budget < 0:
            return False
        # `} else {` and `} else if (...)` first: they open a branch of the
        # chain we are in, and they begin with a brace that would otherwise
        # read as the end of one.
        if re.match(r"\}?\s*else\b", text):
            seen_else = True
            continue
        # Any other closing brace means the construct above has ended, so
        # whatever gate it carried was not ours. Walking past these is what let
        # a slash command borrow the ownership check of the branch that closed
        # just above it, and a dialog borrow the one-liner before it.
        if text.startswith("}"):
            return False
        if text.startswith(("if (", "if(")):
            # A complete one-liner is a sibling statement unless an else binds
            # us to it; an `if` left open governs whatever follows.
            if text.endswith(";") and not seen_else:
                continue
            return _mentions(text, words)
        if text.endswith((";", "{")):
            continue
        return False
    return False
