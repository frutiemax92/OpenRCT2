"""Resolves SPR_* names of src/openrct2/SpriteIds.h to image indices.

Handles the forms the enum uses for the g1/g2 ranges: `NAME,` (previous + 1), `NAME = 123`, `NAME = OTHER`,
`NAME = OTHER + 12` and `NAME = OTHER - 12`. Names defined with other expressions are left out; so is anything
after them until the next resolvable explicit value.
"""

import os
import re

SPRITE_IDS_H = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src", "openrct2", "SpriteIds.h")

_ENTRY = re.compile(r"^\s*(SPR_[A-Z0-9_]+)\s*(?:=\s*([^,]+?))?\s*,?\s*(?://.*)?$")
_TERM = re.compile(r"^(?:(SPR_[A-Z0-9_]+)|(0x[0-9A-Fa-f]+|\d+))(?:\s*([+-])\s*(0x[0-9A-Fa-f]+|\d+))?$")


def _evaluate(expression, values):
    match = _TERM.match(expression.strip())
    if not match:
        return None
    name, literal, op, operand = match.groups()
    if name is not None:
        if name not in values:
            return None
        base = values[name]
    else:
        base = int(literal, 0)
    if op is None:
        return base
    delta = int(operand, 0)
    return base + delta if op == "+" else base - delta


def parse(text):
    """Returns {name: index} for every resolvable enumerator."""
    values = {}
    body = text[text.index("enum : ImageIndex"):]
    body = body[body.index("{") + 1:body.index("\n};")]
    previous = None
    for line in body.splitlines():
        match = _ENTRY.match(line)
        if not match:
            continue
        name, expression = match.groups()
        if expression is None:
            value = None if previous is None else previous + 1
        else:
            value = _evaluate(expression, values)
        if value is not None:
            values[name] = value
        previous = value
    return values


def load(path=SPRITE_IDS_H):
    with open(path, "r", encoding="utf-8") as f:
        return parse(f.read())
