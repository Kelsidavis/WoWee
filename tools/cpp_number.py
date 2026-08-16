"""Read a C++ numeric expression out of checked-in source.

    from cpp_number import number
    number("1.5 / 64.0")   -> 0.0234375
    number("kMaxPieces")   -> None

WHY NOT eval()

The tables these tools read are source files in this repository, so eval()
on them was never reachable by an outsider. It was still eval(), and the
security scan blocks it on sight - correctly, because the argument for it
is about who can reach the input today rather than about the call.

A literal or arithmetic between literals is all the tables have ever held,
and that needs no name lookup, no attribute access and no calls. This walks
the parse tree and accepts only those, so anything else is unreadable rather
than executed, which is what the callers already treat a failure as.
"""
import ast
import re

# 1.5f, 64u, 1L - C++ suffixes that Python's parser does not take.
_SUFFIX = re.compile(r"(?<=[0-9.])[fFuUlL]+\b")

_BINOPS = {
    ast.Add: lambda a, b: a + b,
    ast.Sub: lambda a, b: a - b,
    ast.Mult: lambda a, b: a * b,
    ast.Div: lambda a, b: None if b == 0 else a / b,
}


def _value(node):
    """The node's value, or None if it is anything but literal arithmetic."""
    if isinstance(node, ast.Constant):
        return float(node.value) if isinstance(node.value, (int, float)) \
            and not isinstance(node.value, bool) else None
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        inner = _value(node.operand)
        if inner is None:
            return None
        return inner if isinstance(node.op, ast.UAdd) else -inner
    if isinstance(node, ast.BinOp):
        apply = _BINOPS.get(type(node.op))
        if apply is None:
            return None
        left, right = _value(node.left), _value(node.right)
        if left is None or right is None:
            return None
        return apply(left, right)
    return None


def number(expr):
    """The value of expr as a float, or None if it is not literal arithmetic."""
    if not expr:
        return None
    try:
        tree = ast.parse(_SUFFIX.sub("", expr.strip()), mode="eval")
    except SyntaxError:
        return None
    return _value(tree.body)
