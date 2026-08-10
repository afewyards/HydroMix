#!/usr/bin/env python3
"""Normalise the SnapMagic GCT USB4145-03-0170-C footprint into Kleist2.pretty.

USB4145-03-0170-C is a vertical, top-mount USB 2.0 Type-C receptacle: 16
surface pads, four through-hole shell stakes and two locating pegs. Its 16 pads
map 1:1 onto `Connector:USB_C_Receptacle_USB2.0_16P`, and unlike the USB4115 it
leaves the whole contact row clear of the shell wall, so it can be hand-soldered.
`0170` is the 1.70 mm shell stake length -- correct for a 1.6 mm board. `0070`
and `0230` are the wrong-thickness variants and will not seat.

Two fixes before the vendor file is usable:

1. It carries an Edge.Cuts obround (~1.01 x 0.71 mm) around the RIGHT locating
   peg only, on top of the NPTH pad already there; the left peg has none. A
   closed Edge.Cuts loop inside the board becomes a routed cutout, and 0.71 mm
   is far below any router bit. The inconsistency between the two pegs marks it
   as an export artifact -- GCT's drawing calls out a plain diameter 0.71 hole.
   Dropped; the NPTH pads are authoritative.

2. Shell stakes are named S1..S4. Renamed to `SH` to match the shield pin of
   `Connector:USB_C_Receptacle_USB2.0_16P` and KiCad's own GCT footprints.

WHY THIS SCRIPT EXISTS: the footprint is SnapMagic-derived. Their licence lets
you design, manufacture and distribute *boards* built with the model, but not
redistribute the *model files*. This repo is public and MIT, so the generated
.kicad_mod and .stp are gitignored and you regenerate them here from your own
SnapMagic download. Fabrication outputs -- Gerbers, drill, CPL, BOM -- are board
designs and are explicitly permitted, so a normal CM handoff is unaffected.

The vendor file is KiCad 6 era. Run `kicad-cli fp upgrade` on a copy first, or
pass an already-upgraded file.

Run: python3 tools/import_usb4145.py <vendor.kicad_mod> [vendor.step]
"""

import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PRETTY = os.path.join(ROOT, "pcb", "Kleist2.pretty")
SHAPES = os.path.join(ROOT, "pcb", "3dmodels")

NAME = "USB_C_Receptacle_GCT_USB4145-03-0170-C_Vertical_SMT"
STEP = "usb4145-03-0170-c.stp"
GRAPHICS = ("fp_line", "fp_arc", "fp_circle", "fp_poly", "fp_rect")

# The vendor file ships no descr/tags. Add them here rather than editing the
# board copy, so the placed footprint matches its library version and KiCad's
# lib_footprint_mismatch check stays quiet.
DESCR = ("USB 2.0 Type-C vertical receptacle, GCT USB4145-03-0170-C. "
         "16 surface contacts, 4 through-hole shell stakes, 2 NPTH locating pegs. "
         "0170 = 1.70 mm stake, for a 1.6 mm board. "
         "https://gct.co/files/drawings/usb4145.pdf")
TAGS = "USB C Type-C Receptacle vertical SMD"


class Q(str):
    """A string that was quoted in the source and must be re-quoted on output."""


def parse(text):
    """Parse s-expressions into nested lists. Returns the first top-level node."""
    i, n = 0, len(text)
    stack, cur = [], None
    while i < n:
        c = text[i]
        if c == "(":
            new = []
            if cur is not None:
                cur.append(new)
                stack.append(cur)
            cur = new
            i += 1
        elif c == ")":
            if stack:
                cur = stack.pop()
            else:
                return cur
            i += 1
        elif c.isspace():
            i += 1
        elif cur is None:
            raise ValueError("content before the opening paren at offset %d" % i)
        elif c == '"':
            j, buf = i + 1, []
            while text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j + 1])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            cur.append(Q("".join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in "()\"":
                j += 1
            cur.append(text[i:j])
            i = j
    return cur


def dumps(node, indent=0):
    """Serialise a parsed node back to KiCad-style s-expression text."""
    if isinstance(node, Q):
        esc = node.replace("\\", "\\\\").replace('"', '\\"')
        return '"%s"' % esc
    if isinstance(node, str):
        return node
    pad = "\t" * indent
    head = node[0] if node and isinstance(node[0], str) else None
    if all(not isinstance(x, list) for x in node):
        return pad + "(" + " ".join(dumps(x) for x in node) + ")"
    parts = [pad + "(" + (dumps(head) if head is not None else "")]
    body = node[1:] if head is not None else node
    inline = [x for x in body if not isinstance(x, list)]
    rest = [x for x in body if isinstance(x, list)]
    if inline:
        parts[0] += " " + " ".join(dumps(x) for x in inline)
    for x in rest:
        parts.append(dumps(x, indent + 1))
    parts.append(pad + ")")
    return "\n".join(parts)


def get(node, key):
    """First child list whose head is `key`."""
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src_mod = sys.argv[1]
    src_step = sys.argv[2] if len(sys.argv) > 2 else None
    os.makedirs(PRETTY, exist_ok=True)
    os.makedirs(SHAPES, exist_ok=True)

    root = parse(open(src_mod).read())
    if not root or str(root[0]) != "footprint":
        sys.exit("%s is not a footprint" % src_mod)

    out, cut, renamed = [], 0, 0
    for node in root:
        if isinstance(node, list) and node:
            if node[0] in GRAPHICS:
                layer = get(node, "layer")
                if layer and str(layer[1]) == "Edge.Cuts":
                    cut += 1
                    continue
            if node[0] == "pad" and str(node[1]).startswith("S") \
                    and str(node[1]) != "SH":
                node[1] = Q("SH")
                renamed += 1
            if node[0] == "model":
                continue
        out.append(node)

    out[1] = Q(NAME)
    for k, node in enumerate(out):
        if isinstance(node, list) and node and node[0] == "layer":
            out[k + 1:k + 1] = [["descr", Q(DESCR)], ["tags", Q(TAGS)]]
            break
    if src_step:
        out.append(parse(
            '(model "${KIPRJMOD}/3dmodels/%s"'
            ' (offset (xyz 0 0 0)) (scale (xyz 1 1 1))'
            ' (rotate (xyz 0 0 0)))' % STEP))
        dst_step = os.path.join(SHAPES, STEP)
        if os.path.abspath(src_step) != os.path.abspath(dst_step):
            shutil.copy(src_step, dst_step)

    dst = os.path.join(PRETTY, NAME + ".kicad_mod")
    open(dst, "w").write(dumps(out) + "\n")

    pads = [n for n in out if isinstance(n, list) and n and n[0] == "pad"]
    names = sorted({str(p[1]) for p in pads})
    print("dropped %d Edge.Cuts elements, renamed %d stake pads" % (cut, renamed))
    print("%d pads: %s" % (len(pads), " ".join(names)))
    print("wrote", os.path.relpath(dst, ROOT))


if __name__ == "__main__":
    main()
