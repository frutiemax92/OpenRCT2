#!/usr/bin/env python3
"""REN-053: no string literal the player can see may contain the word "park".

Scans the string literals of the UI, the golf code and the few files that write player-visible text directly.
File names and ids (".park", "interface.open.park", "*.park") are not words and are skipped.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
FILES = [
    *sorted((ROOT / "src" / "openrct2-ui").rglob("*.cpp")),
    *sorted((ROOT / "src" / "openrct2" / "golf").rglob("*.cpp")),
    ROOT / "src" / "openrct2" / "Game.cpp",
    ROOT / "src" / "openrct2" / "platform" / "Platform.Win32.cpp",
    ROOT / "src" / "openrct2" / "interface" / "InteractiveConsole.cpp",
]
LITERAL = re.compile(r'"((?:[^"\\\n]|\\.)*)"')
WORD = re.compile(r"(?<![\w.*/_-])park(?:s|'s|’s)?(?![\w.*/_-])", re.IGNORECASE)


def strip_comments(line):
    # Good enough for this scan: drop // comments that are not inside a literal.
    out, in_string, i = [], False, 0
    while i < len(line):
        c = line[i]
        if in_string:
            out.append(c)
            if c == "\\":
                out.append(line[i + 1 : i + 2])
                i += 1
            elif c == '"':
                in_string = False
        elif c == '"':
            in_string = True
            out.append(c)
        elif line.startswith("//", i):
            break
        else:
            out.append(c)
        i += 1
    return "".join(out)


def main():
    failures = []
    for path in FILES:
        if not path.exists():
            continue
        in_block_comment = False
        for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            stripped = line.strip()
            if in_block_comment:
                in_block_comment = "*/" not in stripped
                continue
            if stripped.startswith("/*"):
                in_block_comment = "*/" not in stripped
                continue
            if stripped.startswith("#include") or stripped.startswith("*"):
                continue
            for literal in LITERAL.findall(strip_comments(line)):
                if WORD.search(literal):
                    failures.append(f"{path.relative_to(ROOT)}:{number}: {literal}")
    for failure in failures:
        print(failure)
    if failures:
        print(f"{len(failures)} string literal(s) still contain the word 'park'")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
