"""Minimal YAML reader/writer for the linuxcnc-ethercat maintainer scripts.

Only handles the subset emitted by these scripts:
  - flat string-keyed mappings, optionally with a leading ``---`` doc start
  - lists of flat string-keyed mappings (``- key: value`` blocks)

Strings only; no integers, booleans, anchors, multi-line scalars, or nesting.
Quoting is bare or double-quoted (``"...";`` no escapes inside the strings we
emit). Anything outside that subset is undefined behavior — keeping the parser
small is the whole point of not pulling in PyYAML.
"""

import re

_HEX_RE = re.compile(r"^0x[0-9a-fA-F]+$")
_INT_RE = re.compile(r"^-?\d+$")


def _unquote(s):
    if len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        return s[1:-1]
    return s


def load(text):
    """Parse YAML text into a dict or list-of-dicts."""
    items = []  # list of dicts (when input is a sequence)
    single = {}  # for non-list input
    is_list = False
    current = None

    for raw in text.splitlines():
        line = raw.rstrip()
        if not line or line.startswith("#") or line.startswith("---") or line.startswith("..."):
            continue

        if line.startswith("- "):
            is_list = True
            current = {}
            items.append(current)
            line = line[2:]
        elif line.startswith("  ") and is_list:
            line = line[2:]
        else:
            current = single

        if ":" not in line:
            continue
        k, _, v = line.partition(":")
        current[k.strip()] = _unquote(v.strip())

    return items if is_list else single


def quote_scalar(s):
    """Render a string scalar with the same conservative quoting Go's
    yaml.v3 used in this project's fixtures: empty strings, hex literals,
    and bare-integer-looking strings get double-quoted; everything else
    is left bare."""
    if s == "":
        return '""'
    if _HEX_RE.match(s) or _INT_RE.match(s):
        return f'"{s}"'
    return s


def dump_mapping(d, out, doc_start=True):
    """Write a single mapping in block style. Field order = dict order."""
    if doc_start:
        out.write("---\n")
    for k, v in d.items():
        out.write(f"{k}: {quote_scalar(str(v))}\n")


def dump_sequence(items, out):
    """Write a list of mappings in block style (``- k: v`` blocks)."""
    for d in items:
        first = True
        for k, v in d.items():
            prefix = "- " if first else "  "
            out.write(f"{prefix}{k}: {quote_scalar(str(v))}\n")
            first = False
