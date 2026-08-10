#!/usr/bin/env python3
"""Read the U7 (IS31FL3730) -> DS1 (KWM-20881AGB) mapping out of the design and check it.

READ-ONLY. The design files are the source of truth; nothing here is hardcoded, so
a hand-edited assignment is reported, never overwritten. Run it after touching the
display wiring, and before writing the framebuffer tables into firmware.

What it checks
--------------
1. Schematic and PCB agree on every DISP_* net.
2. The source/sink rule (the one that silently kills the display).

   IS31FL3730 datasheet Rev. D, Application Information + Config Register 00h:

     DM=00  Matrix 1 only  - "Matrix 1 LED columns have common cathodes and are
                             connected to the C1:C8 outputs. The rows are
                             connected to the row drivers."
     DM=01  Matrix 2 only  - "Matrix 2 LED rows have common cathodes and
                             connected to the R1:R8."

   The KWM-20881AGB is common-ROW-anode. So exactly two whole-group wirings are
   legal, and firmware must be told which:

     Matrix 1 (DM=00): panel ROW pins -> R1..R8   panel COL pins -> C1..C8
     Matrix 2 (DM=01): panel ROW pins -> C1..C8   panel COL pins -> R1..R8

   Permuting freely WITHIN a group is fine and is how the fan-out was made
   routable. MIXING the groups is not: it ties a current source to a cathode and
   a sink to an anode, reverse-biasing every LED. The board looks fine, DRC and
   ERC pass, and nothing lights.

3. Routability, scored on the real topology: nets cannot cross the panel, so the
   fan-out is planar iff the order nets leave U7's perimeter matches the order of
   pins around the panel. Reports crossings and, more usefully, how many nets need
   a via pair to B.Cu (16 - longest increasing subsequence).

Exit status is non-zero if any check fails, so it can gate a commit.

Usage: python3 tools/display_map.py [--firmware-table]
"""
import bisect
import math
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PCB = os.path.join(ROOT, "pcb", "ValveController.kicad_pcb")
SCH = os.path.join(ROOT, "pcb", "ValveController.kicad_sch")

DRV_RE = r'(R\d|C\d)(?:/C\d+)?_\d+$'
PANEL_RE = r'(ROW\d|COL\d)_\d+$'


def _span(s, i):
    d = 0
    j = i
    while True:
        if s[j] == '(':
            d += 1
        elif s[j] == ')':
            d -= 1
            if d == 0:
                return j + 1
        j += 1


def footprint(src, ref):
    m = re.search(r'\(property "Reference" "%s"\n' % ref, src)
    if not m:
        sys.exit("footprint %s not found" % ref)
    i = src.rfind('\n\t(footprint', 0, m.start())
    return src[i:_span(src, i + 1)]


def pads(src, ref, pat):
    """{logical pin name: (net, x, y)} in board coordinates."""
    b = footprint(src, ref)
    at = re.search(r'\n\t\t\(at ([\d.-]+) ([\d.-]+)(?: ([\d.-]+))?\)', b)
    ox, oy, rot = float(at.group(1)), float(at.group(2)), float(at.group(3) or 0)
    a = math.radians(-rot)
    out = {}
    k = 0
    while True:
        pm = re.search(r'\(pad "([^"]*)"', b[k:])
        if not pm:
            break
        ps = k + pm.start()
        pe = _span(b, ps)
        blk = b[ps:pe]
        fn = re.search(r'\(pinfunction "([^"]*)"\)', blk)
        nm = re.search(r'\(net "([^"]*)"\)', blk)
        pa = re.search(r'\(at ([\d.-]+) ([\d.-]+)', blk)
        if fn and nm:
            mm = re.match(pat, fn.group(1))
            if mm:
                lx, ly = float(pa.group(1)), float(pa.group(2))
                out[mm.group(1)] = (nm.group(1),
                                    ox + lx * math.cos(a) - ly * math.sin(a),
                                    oy + lx * math.sin(a) + ly * math.cos(a))
        k = pe
    return out, (ox, oy, rot)


def sch_ds1_labels(src):
    """DS1-side global labels, keyed by panel pin, read via the symbol's pin order."""
    b = footprint(open(PCB).read(), 'DS1')
    order = []
    k = 0
    while True:
        pm = re.search(r'\(pad "(\d+)"', b[k:])
        if not pm:
            break
        ps = k + pm.start()
        pe = _span(b, ps)
        fn = re.search(r'\(pinfunction "((?:ROW|COL)\d)_\d+"\)', b[ps:pe])
        if fn:
            order.append((int(pm.group(1)), fn.group(1)))
        k = pe
    order.sort()
    m = re.search(r'\(property "Reference" "DS1"\n', src)
    i = src.rfind('\n\t(symbol', 0, m.start())
    sx = re.search(r'\(at ([\d.-]+) ([\d.-]+)', src[i:i + 200])
    x0, y0 = float(sx.group(1)), float(sx.group(2))
    out = {}
    for idx, (_, pin) in enumerate(order):
        y = y0 + 2.54 * idx
        lm = re.search(r'\(global_label "(DISP_[RC]\d)"\n\t\t\(shape \w+\)\n'
                       r'\t\t\(at %.2f %.2f ' % (x0, y), src)
        if lm:
            out[pin] = lm.group(1)
    return out


def inversions(seq):
    n = 0
    for i in range(len(seq)):
        for j in range(i + 1, len(seq)):
            if seq[i] > seq[j]:
                n += 1
    return n


def lis(seq):
    t = []
    for v in seq:
        i = bisect.bisect_left(t, v)
        if i == len(t):
            t.append(v)
        else:
            t[i] = v
    return len(t)


def main():
    pcb = open(PCB).read()
    sch = open(SCH).read()
    u7, u7at = pads(pcb, 'U7', DRV_RE)
    ds1, _ = pads(pcb, 'DS1', PANEL_RE)
    fail = []

    net2pin = {v[0]: k for k, v in ds1.items()}
    mapping = {d: net2pin.get(v[0]) for d, v in u7.items()}
    missing = [d for d, p in mapping.items() if p is None]
    if missing:
        fail.append("driver outputs with no panel pin: %s" % ", ".join(sorted(missing)))

    # 1. schematic vs pcb
    labels = sch_ds1_labels(sch)
    for pin, net in labels.items():
        if pin in ds1 and ds1[pin][0] != net:
            fail.append("%s: schematic says %s, PCB says %s" % (pin, net, ds1[pin][0]))

    # 2. source/sink rule
    pairs = [(d, p) for d, p in mapping.items() if p]
    m1 = [(d, p) for d, p in pairs if (d[0] == 'R') != p.startswith('ROW')]
    m2 = [(d, p) for d, p in pairs if (d[0] == 'R') == p.startswith('ROW')]
    if not m1:
        mode = "Matrix 1 only (DM=00) - panel rows on R1:R8"
    elif not m2:
        mode = "Matrix 2 only (DM=01) - panel rows on C1:C8"
    else:
        mode = None
        fail.append("source/sink groups are MIXED - %d of %d pins cross the "
                    "boundary. Legal wirings are all-rows-to-R (DM=00) or "
                    "all-rows-to-C (DM=01); mixing reverse-biases every LED "
                    "and nothing lights." % (min(len(m1), len(m2)), len(pairs)))
        for d, p in sorted(m1 if len(m1) < len(m2) else m2):
            fail.append("    %s -> %s" % (d, p))

    print("U7 at (%.3f, %.3f) rot %g   DS1 %d pins" % (u7at[0], u7at[1], u7at[2], len(ds1)))
    if mode:
        print("wiring orientation: %s" % mode)
        print("  -> firmware MUST set Configuration Register 00h DM accordingly")

    # 3. routability
    cx = sum(v[1] for v in ds1.values()) / len(ds1)
    cy = sum(v[2] for v in ds1.values()) / len(ds1)
    ring = {m: i for i, m in enumerate(sorted(
        ds1, key=lambda m: math.degrees(math.atan2(-(ds1[m][1] - cx), -(ds1[m][2] - cy))) % 360))}
    order = sorted((d for d, p in pairs),
                   key=lambda d: (-math.degrees(math.atan2(u7[d][1] - u7at[0],
                                                           u7[d][2] - u7at[1]))) % 360)
    seq = [ring[mapping[d]] for d in order]
    print("routability: %d crossings, %d nets need a via pair to B.Cu"
          % (inversions(seq), len(seq) - lis(seq)))

    if '--firmware-table' in sys.argv and mode:
        inv = {p: d for d, p in pairs}
        print("\npanel -> driver (framebuffer lookup):")
        for grp in ('ROW', 'COL'):
            print("  " + "  ".join("%s=%s" % (g, inv[g])
                                   for g in sorted((p for p in inv if p.startswith(grp)),
                                                   key=lambda s: int(s[3:]))))

    if fail:
        print("\nFAIL")
        for f in fail:
            print("  " + f)
        return 1
    print("\nOK - schematic and PCB agree, source/sink rule satisfied")
    return 0


if __name__ == "__main__":
    sys.exit(main())
