#!/usr/bin/env python3
"""Generate KiCad footprints for board artwork: the Kleist Labs logo and a URL QR code.

KiCad's fp_poly has no notion of holes, so any shape with holes (the logo body with
the K knocked out, letters, QR finder patterns) has to be fractured into simple
outlines joined by zero-width bridges -- the same thing KiCad does internally when
it saves a SHAPE_POLY_SET. That is what fracture() below does.

Run it with:
    uv run --with pyclipper --with segno --with potracer --with numpy --with pillow \
        tools/make_pcb_art.py
"""

from __future__ import annotations

import argparse
import math
import re
import uuid
from pathlib import Path

import numpy as np
import potrace
import pyclipper
import segno
from PIL import Image, ImageDraw, ImageFilter

CLIP = 10_000  # pyclipper integer scale
NS = uuid.UUID("6f1f5c1e-2b7a-5a3d-9c44-8b1c0d7e4a11")  # stable uuid5 namespace

LAYER_VARIANTS = {
    "SilkScreen": (["F.SilkS"], "silkscreen"),
    "SolderMask": (["F.Mask"], "solder mask opening"),
    "Copper": (["F.Cu", "F.Mask"], "exposed copper"),
}

# source file, footprint/symbol prefix, description, layer variants, sizes in mm.
# The first size is primary and gets bare symbol names.
LOGOS = [
    # The tab-to-frame gap is the mark's tightest feature at 3.75% of its width, so
    # it holds 337 um at 9 mm and 300 um at 8 mm -- twice the 150 um silk minimum.
    # The tabs run to the edge of the source canvas, so "width" here spans tab tip
    # to tab tip, not the frame.
    ("kleist-labs-12a-8mm.png", "KleistLabs_Logo", "Kleist Labs logo",
     ("SilkScreen", "SolderMask", "Copper"), (9.0, 8.0)),
    # Solid mark, no tabs: smallest gap is 913 um at 10 mm, so it shrinks happily.
    ("kleist-labs-monogram.png", "KleistLabs_Monogram", "Kleist Labs monogram",
     ("SilkScreen", "Copper"), (10.0, 6.0)),
]


# --------------------------------------------------------------------------- SVG

def parse_path(d: str, arc_tol: float = 0.02) -> list[list[tuple[float, float]]]:
    """Flatten an SVG path into closed subpaths. Supports M/L/H/V/A/Z, absolute."""
    tokens = re.findall(r"[MmLlHhVvAaZz]|-?[\d.]+(?:e-?\d+)?", d)
    subpaths: list[list[tuple[float, float]]] = []
    cur: list[tuple[float, float]] = []
    x = y = 0.0
    i = 0
    cmd = ""
    while i < len(tokens):
        t = tokens[i]
        if re.match(r"[A-Za-z]", t):
            cmd = t
            i += 1
        if cmd in "Zz":
            if cur:
                subpaths.append(cur)
                cur = []
            continue
        if cmd == "M":
            if cur:
                subpaths.append(cur)
            x, y = float(tokens[i]), float(tokens[i + 1])
            cur = [(x, y)]
            i += 2
            cmd = "L"  # implicit lineto for repeated coordinate pairs
        elif cmd == "L":
            x, y = float(tokens[i]), float(tokens[i + 1])
            cur.append((x, y))
            i += 2
        elif cmd == "H":
            x = float(tokens[i])
            cur.append((x, y))
            i += 1
        elif cmd == "V":
            y = float(tokens[i])
            cur.append((x, y))
            i += 1
        elif cmd == "A":
            rx, ry, rot = float(tokens[i]), float(tokens[i + 1]), float(tokens[i + 2])
            fa, fs = int(float(tokens[i + 3])), int(float(tokens[i + 4]))
            nx, ny = float(tokens[i + 5]), float(tokens[i + 6])
            cur.extend(arc_points(x, y, rx, ry, rot, fa, fs, nx, ny, arc_tol))
            x, y = nx, ny
            i += 7
        else:
            raise ValueError(f"unsupported path command {cmd!r}")
    if cur:
        subpaths.append(cur)
    return subpaths


def arc_points(x1, y1, rx, ry, phi_deg, fa, fs, x2, y2, tol):
    """SVG endpoint-parameterised arc -> polyline (excluding the start point)."""
    if rx == 0 or ry == 0 or (x1 == x2 and y1 == y2):
        return [(x2, y2)]
    phi = math.radians(phi_deg)
    cp, sp = math.cos(phi), math.sin(phi)
    dx2, dy2 = (x1 - x2) / 2.0, (y1 - y2) / 2.0
    x1p, y1p = cp * dx2 + sp * dy2, -sp * dx2 + cp * dy2
    rx, ry = abs(rx), abs(ry)
    lam = (x1p / rx) ** 2 + (y1p / ry) ** 2
    if lam > 1:
        s = math.sqrt(lam)
        rx, ry = rx * s, ry * s
    num = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p
    den = rx * rx * y1p * y1p + ry * ry * x1p * x1p
    co = math.sqrt(max(0.0, num / den))
    if fa == fs:
        co = -co
    cxp, cyp = co * rx * y1p / ry, -co * ry * x1p / rx
    cx, cy = cp * cxp - sp * cyp + (x1 + x2) / 2, sp * cxp + cp * cyp + (y1 + y2) / 2

    def ang(ux, uy, vx, vy):
        d = math.hypot(ux, uy) * math.hypot(vx, vy)
        a = math.acos(max(-1.0, min(1.0, (ux * vx + uy * vy) / d)))
        return -a if ux * vy - uy * vx < 0 else a

    th1 = ang(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry)
    dth = ang((x1p - cxp) / rx, (y1p - cyp) / ry, (-x1p - cxp) / rx, (-y1p - cyp) / ry)
    if not fs and dth > 0:
        dth -= 2 * math.pi
    elif fs and dth < 0:
        dth += 2 * math.pi

    r = max(rx, ry)
    step = 2 * math.acos(max(-1.0, min(1.0, 1 - tol / r))) if r > tol else math.pi / 4
    n = max(2, math.ceil(abs(dth) / max(step, 1e-6)))
    out = []
    for k in range(1, n + 1):
        th = th1 + dth * k / n
        ex, ey = rx * math.cos(th), ry * math.sin(th)
        out.append((cp * ex - sp * ey + cx, sp * ex + cp * ey + cy))
    out[-1] = (x2, y2)
    return out


# ----------------------------------------------------------------- boolean + fracture

def evenodd_regions(subpaths):
    """Resolve SVG even-odd fill into (outer, holes) groups."""
    pc = pyclipper.Pyclipper()
    pc.AddPaths(
        [[(round(x * CLIP), round(y * CLIP)) for x, y in sp] for sp in subpaths],
        pyclipper.PT_SUBJECT,
        True,
    )
    tree = pc.Execute2(pyclipper.CT_UNION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
    groups = []

    def walk(node):
        for child in node.Childs:
            if child.IsHole:
                walk(child)  # islands inside a hole are outers one level down
            else:
                holes = [h.Contour for h in child.Childs if h.IsHole]
                groups.append((child.Contour, holes))
                walk(child)

    walk(tree)
    return groups


def fracture(outer, holes):
    """Merge holes into their outline with zero-width bridges -> one simple polygon."""
    poly = [tuple(p) for p in outer]
    todo = [[tuple(p) for p in h] for h in holes]
    while todo:
        # rightmost hole vertex first: its +x ray can only hit already-merged geometry
        hi, vi = max(
            ((i, j) for i, h in enumerate(todo) for j in range(len(h))),
            key=lambda t: todo[t[0]][t[1]][0],
        )
        hole = todo.pop(hi)
        px, py = hole[vi]
        best_e, best_x = None, None
        for e in range(len(poly)):
            ax, ay = poly[e]
            bx, by = poly[(e + 1) % len(poly)]
            if (ay > py) == (by > py):
                continue
            ix = ax + (py - ay) * (bx - ax) / (by - ay)
            if ix > px and (best_x is None or ix < best_x):
                best_e, best_x = e, ix
        if best_e is None:
            raise RuntimeError("fracture: no bridge target found")
        q = (best_x, py)
        bridge = [q] + hole[vi:] + hole[:vi] + [hole[vi], q]
        poly = poly[: best_e + 1] + bridge + poly[best_e + 1 :]
    return poly


def shapes_from_subpaths(subpaths):
    """Even-odd fill -> hole-free polygons, back in SVG user units."""
    return [
        [(x / CLIP, y / CLIP) for x, y in fracture(o, h)]
        for o, h in evenodd_regions(subpaths)
    ]


# --------------------------------------------------------------------- KiCad output

def fp_poly(points, layer, key, width=0.01):
    pts = " ".join(f"(xy {x:.6f} {y:.6f})" for x, y in points)
    u = uuid.uuid5(NS, key)
    return (
        f"\t(fp_poly\n\t\t(pts\n\t\t\t{pts}\n\t\t)\n"
        f"\t\t(stroke\n\t\t\t(width {width})\n\t\t\t(type solid)\n\t\t)\n"
        f"\t\t(fill yes)\n\t\t(layer \"{layer}\")\n"
        f'\t\t(uuid "{u}")\n\t)\n'
    )


def footprint(name, shapes, layers, descr, tags, ref_y):
    def prop(pname, value, layer, y, size=1.0):
        u = uuid.uuid5(NS, f"{name}:{pname}")
        return (
            f'\t(property "{pname}" "{value}"\n\t\t(at 0 {y:.3f} 0)\n'
            f'\t\t(layer "{layer}")\n\t\t(hide yes)\n\t\t(uuid "{u}")\n'
            f"\t\t(effects\n\t\t\t(font\n\t\t\t\t(size {size} {size})\n"
            f"\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)\n"
        )

    out = [
        f'(footprint "{name}"\n\t(version 20260206)\n\t(generator "make_pcb_art")\n'
        f'\t(generator_version "10.0")\n\t(layer "F.Cu")\n\t(descr "{descr}")\n'
        f'\t(tags "{tags}")\n'
    ]
    out.append(prop("Reference", "REF**", "F.SilkS", -ref_y))
    out.append(prop("Value", name, "F.Fab", ref_y))
    out.append(prop("Datasheet", "", "F.Fab", 0, 1.27))
    out.append(prop("Description", "", "F.Fab", 0, 1.27))
    out.append(
        "\t(attr exclude_from_pos_files exclude_from_bom allow_missing_courtyard)\n"
        "\t(duplicate_pad_numbers_are_jumpers no)\n"
    )
    for layer in layers:
        for i, s in enumerate(shapes):
            out.append(fp_poly(s, layer, f"{name}:{layer}:{i}"))
    out.append("\t(embedded_fonts no)\n)\n")
    return "".join(out)


def to_mm(shapes, scale, cx, cy):
    return [[((x - cx) * scale, (y - cy) * scale) for x, y in s] for s in shapes]


# ------------------------------------------------------------------ symbol output

def sym_prop(name, value, x, y, hide=True):
    return (
        f'\t\t(property "{name}" "{value}"\n\t\t\t(at {x:g} {y:g} 0)\n'
        f"\t\t\t(show_name no)\n\t\t\t(do_not_autoplace no)\n"
        f"\t\t\t(hide {'yes' if hide else 'no'})\n"
        f"\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n"
    )


def symbol(name, shapes, size_mm, fp, descr):
    """Graphic-only symbol. No pins, so it adds no nets -- it exists purely to carry
    the footprint onto the board from the schematic."""
    xs = [x for s in shapes for x, _ in s]
    ys = [y for s in shapes for _, y in s]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    k = size_mm / span
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    half = size_mm / 2

    out = [
        f'\t(symbol "{name}"\n\t\t(exclude_from_sim yes)\n\t\t(in_bom no)\n'
        f"\t\t(on_board yes)\n\t\t(in_pos_files no)\n"
        f"\t\t(duplicate_pin_numbers_are_jumpers no)\n"
    ]
    out.append(sym_prop("Reference", "G", 0, half + 1.27))
    out.append(sym_prop("Value", name, 0, -half - 1.27))
    out.append(sym_prop("Footprint", fp, 0, 0))
    out.append(sym_prop("Datasheet", "", 0, 0))
    out.append(sym_prop("Description", descr, 0, 0))
    out.append(sym_prop("ki_keywords", "logo graphic artwork", 0, 0))
    out.append(f'\t\t(symbol "{name}_0_1"\n')
    for s in shapes:
        # symbol space has +Y up, the opposite of board space
        pts = " ".join(f"(xy {(x-cx)*k:.4f} {-(y-cy)*k:.4f})" for x, y in s)
        out.append(
            f"\t\t\t(polyline\n\t\t\t\t(pts\n\t\t\t\t\t{pts}\n\t\t\t\t)\n"
            f"\t\t\t\t(stroke\n\t\t\t\t\t(width 0.001)\n\t\t\t\t\t(type solid)\n\t\t\t\t)\n"
            f"\t\t\t\t(fill\n\t\t\t\t\t(type outline)\n\t\t\t\t)\n\t\t\t)\n"
        )
    out.append("\t\t)\n\t\t(embedded_fonts no)\n\t)\n")
    return "".join(out)


def sym_lib(symbols):
    return (
        '(kicad_symbol_lib\n\t(version 20251024)\n\t(generator "make_pcb_art")\n'
        '\t(generator_version "10.0")\n' + "".join(symbols) + ")\n"
    )


# -------------------------------------------------------------------------- logo

def cubic(p0, p1, p2, p3, tol_px):
    """Flatten a cubic bezier, segment count from the control polygon length."""
    ctrl = sum(math.dist(a, b) for a, b in ((p0, p1), (p1, p2), (p2, p3)))
    n = max(2, min(24, math.ceil(ctrl / max(tol_px * 8, 1e-6))))
    out = []
    for i in range(1, n + 1):
        t = i / n
        u = 1 - t
        out.append((
            u**3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t**3 * p3[0],
            u**3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t**3 * p3[1],
        ))
    return out


def trace_png(png_file, turdsize=2, alphamax=1.0, tol_px=0.4):
    """Vectorise a black-on-white (or transparent) bitmap into closed subpaths."""
    img = Image.open(png_file).convert("RGBA")
    flat = Image.new("RGB", img.size, "white")
    flat.paste(img, mask=img.split()[-1])
    ink = np.asarray(flat.convert("L")) < 128
    # potrace here treats falsy as foreground, so hand it the inverse -- otherwise
    # it traces the background and returns the whole canvas as one big contour.
    path = potrace.Bitmap(~ink).trace(turdsize=turdsize, alphamax=alphamax)

    subpaths = []
    for curve in path.curves:
        start = (curve.start_point.x, curve.start_point.y)
        pts = [start]
        cur = start
        for seg in curve.segments:
            end = (seg.end_point.x, seg.end_point.y)
            if seg.is_corner:
                pts.append((seg.c.x, seg.c.y))
                pts.append(end)
            else:
                pts.extend(cubic(cur, (seg.c1.x, seg.c1.y), (seg.c2.x, seg.c2.y),
                                 end, tol_px))
            cur = end
        subpaths.append(pts)
    return subpaths


def load_logo(src, width_mm, min_gap_mm):
    """Return (subpaths, scale, notes). Scale maps source units -> mm so that the
    artwork's own bounding box, not the canvas, is width_mm across."""
    src = Path(src)
    notes = []
    if src.suffix.lower() == ".svg":
        m = re.search(r'<path[^>]*\sd="([^"]+)"', src.read_text())
        if not m:
            raise SystemExit(f"{src}: no <path d=...> found")
        subpaths = parse_path(m.group(1))
    else:
        subpaths = trace_png(src)
        notes.append(f"traced {len(subpaths)} contours from bitmap")

    xs = [x for sp in subpaths for x, _ in sp]
    ys = [y for sp in subpaths for _, y in sp]
    extent = max(max(xs) - min(xs), max(ys) - min(ys))
    scale = width_mm / extent

    if src.suffix.lower() == ".svg":
        # The SVG's dot sits 1 unit from the K stem; open that up to a printable gap.
        arced = [i for i, sp in enumerate(subpaths) if len(sp) > 8]
        dot = min(arced, key=lambda i: bbox_area(subpaths[i]))
        gap = min(x for x, _ in subpaths[dot + 1]) - max(x for x, _ in subpaths[dot])
        shift = max(0.0, min_gap_mm / scale - gap)
        if shift:
            subpaths[dot] = [(x - shift, y) for x, y in subpaths[dot]]
            notes.append(f"dot shifted {shift:.2f} units to open the stem gap to "
                         f"{min_gap_mm*1000:.0f} um")
    return subpaths, scale, notes


def bbox_area(sp):
    xs = [p[0] for p in sp]
    ys = [p[1] for p in sp]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))


def dfm_report(shapes_mm, min_mm, px_per_mm=60):
    """Measure how much artwork is thinner than min_mm, and how much of the gaps
    between artwork are narrower than min_mm, by morphological opening/closing."""
    xs = [x for s in shapes_mm for x, _ in s]
    ys = [y for s in shapes_mm for _, y in s]
    pad = 1.0
    W = int((max(xs) - min(xs) + 2 * pad) * px_per_mm)
    H = int((max(ys) - min(ys) + 2 * pad) * px_per_mm)
    img = Image.new("L", (W, H), 0)
    d = ImageDraw.Draw(img)
    for s in shapes_mm:
        d.polygon([((x - min(xs) + pad) * px_per_mm, (y - min(ys) + pad) * px_per_mm)
                   for x, y in s], fill=255)
    k = max(3, int(round(min_mm * px_per_mm)) | 1)
    ink = np.asarray(img) > 127
    opened = np.asarray(img.filter(ImageFilter.MinFilter(k)).filter(
        ImageFilter.MaxFilter(k))) > 127
    closed = np.asarray(img.filter(ImageFilter.MaxFilter(k)).filter(
        ImageFilter.MinFilter(k))) > 127
    px_mm2 = 1.0 / (px_per_mm ** 2)
    return {
        "ink_mm2": ink.sum() * px_mm2,
        "thin_mm2": (ink & ~opened).sum() * px_mm2,   # silk narrower than min_mm
        "pinch_mm2": (closed & ~ink).sum() * px_mm2,  # gaps narrower than min_mm
        "kernel_mm": k / px_per_mm,
    }


# ---------------------------------------------------------------------------- QR

def qr_rects(matrix, draw_light, quiet):
    n = len(matrix)
    size = n + 2 * quiet
    grid = [[False] * size for _ in range(size)]
    for r in range(size):
        for c in range(size):
            inside = quiet <= r < quiet + n and quiet <= c < quiet + n
            dark = inside and matrix[r - quiet][c - quiet]
            grid[r][c] = (not dark) if draw_light else dark

    runs = []
    for row in grid:
        rr, c = [], 0
        while c < size:
            if row[c]:
                c0 = c
                while c < size and row[c]:
                    c += 1
                rr.append((c0, c))
            else:
                c += 1
        runs.append(rr)

    used = [[False] * len(r) for r in runs]
    rects = []
    for r in range(size):
        for i, run in enumerate(runs[r]):
            if used[r][i]:
                continue
            r2 = r + 1
            while r2 < size:
                hit = next(
                    (j for j, o in enumerate(runs[r2]) if o == run and not used[r2][j]),
                    None,
                )
                if hit is None:
                    break
                used[r2][hit] = True
                r2 += 1
            rects.append((run[0], r, run[1], r2))
    return rects, size


# -------------------------------------------------------------------------- main

def preview(path, shapes_by_colour, w, h, bg="#1b6b3a"):
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
        f'width="{w*4}" height="{h*4}"><rect width="{w}" height="{h}" fill="{bg}"/>'
    ]
    for colour, shapes in shapes_by_colour:
        for s in shapes:
            pts = " ".join(f"{x:.4f},{y:.4f}" for x, y in s)
            parts.append(f'<polygon points="{pts}" fill="{colour}" fill-rule="nonzero"/>')
    parts.append("</svg>")
    Path(path).write_text("".join(parts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--art-dir", default="pcb/art", help="where LOGOS sources live")
    ap.add_argument("--outdir", default="pcb/Kleist2.pretty")
    ap.add_argument("--preview-dir", default="pcb/art")
    ap.add_argument("--logo-widths", type=float, nargs="+", default=None,
                    help="mm across, overrides the per-logo sizes in LOGOS")
    ap.add_argument("--min-gap", type=float, default=0.15, help="mm, min silk feature")
    ap.add_argument("--url", default="https://github.com/afewyards/HydroMix")
    ap.add_argument("--qr-module", type=float, default=0.25, help="mm per QR module")
    ap.add_argument("--qr-max", type=float, default=10.0,
                    help="mm, warn if a QR ends up larger than this")
    ap.add_argument("--qr-ecc", default="m", choices=list("lmqh"))
    ap.add_argument("--qr-quiet", type=int, default=4, help="quiet zone, modules")
    ap.add_argument("--sym-lib", default="pcb/Kleist2.kicad_sym")
    ap.add_argument("--sym-size", type=float, default=12.7, help="mm, drawing on sheet")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    prevdir = Path(args.preview_dir)
    prevdir.mkdir(parents=True, exist_ok=True)

    nick = outdir.stem  # footprint library nickname, e.g. "Kleist2"
    symbols = []

    for srcname, prefix, title, wanted, sizes in LOGOS:
        src = Path(args.art_dir) / srcname
        if not src.exists():
            print(f"logo: {srcname} not found, skipped")
            continue
        for i, width in enumerate(args.logo_widths or sizes):
            tag = f"{width:g}mm".replace(".", "v")
            subpaths, scale, notes = load_logo(src, width, args.min_gap)
            shapes = shapes_from_subpaths(subpaths)
            xs = [p[0] for s in shapes for p in s]
            ys = [p[1] for s in shapes for p in s]
            cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            art = to_mm(shapes, scale, cx, cy)
            half = max(max(abs(x) for s in art for x, _ in s),
                       max(abs(y) for s in art for _, y in s)) + 0.8
            # the primary size keeps the bare symbol name, so symbols already
            # placed on a schematic keep resolving when extra sizes are added
            sym_tag = "" if i == 0 else f"_{tag}"

            for suffix in wanted:
                layers, what = LAYER_VARIANTS[suffix]
                name = f"{prefix}_{tag}_{suffix}"
                descr = f"{title} {tag}, {what}"
                (outdir / f"{name}.kicad_mod").write_text(
                    footprint(name, art, layers, descr, "logo", half)
                )
                symbols.append(
                    symbol(f"{prefix}{sym_tag}_{suffix}", art, args.sym_size,
                           f"{nick}:{name}", descr)
                )

            r = dfm_report(art, args.min_gap)
            print(f"logo: {srcname} @ {width:g} mm -> {len(shapes)} polygons, "
                  f"variants {'+'.join(wanted)}")
            for note in notes:
                print(f"      {note}")
            thin_pc = 100 * r["thin_mm2"] / r["ink_mm2"]
            pinch_pc = 100 * r["pinch_mm2"] / r["ink_mm2"]
            print(f"      dfm @ {r['kernel_mm']*1000:.0f} um: artwork thinner than "
                  f"that {thin_pc:.2f}%, gaps narrower {pinch_pc:.2f}% "
                  f"(of {r['ink_mm2']:.2f} mm2 ink)")
            # A percent or so is just concave corners. Well past that means real
            # features sit at or under the limit -- note the raster quantises to
            # about +/-17 um, so a flagged mark may be exactly at the floor rather
            # than under it. Measure before deciding it is unusable.
            for what, v in (("features", thin_pc), ("gaps", pinch_pc)):
                if v > 1.0:
                    print(f"      WARNING: {v:.1f}% of {what} are at or below "
                          f"{args.min_gap*1000:.0f} um -- no margin, check the fab spec")
            preview(prevdir / f"preview_{prefix}_{tag}.svg",
                    [("#ffffff", [[(x + half, y + half) for x, y in p] for p in art])],
                    half * 2, half * 2)

    # ---- QR
    qr = segno.make(args.url, error=args.qr_ecc, boost_error=False)
    matrix = [[bool(v) for v in row] for row in qr.matrix]
    n = len(matrix)
    # SilkOnDark prints the dark modules, so its quiet zone is just bare board and
    # costs no footprint area. DarkOnSilk prints the light modules, so the quiet
    # zone has to be silk and does count against the size budget.
    for polarity, draw_light in (("SilkOnDark", False), ("DarkOnSilk", True)):
        quiet = args.qr_quiet if draw_light else 0
        # Pitch is set explicitly rather than derived from a size budget: decode
        # robustness tracks bleed/pitch, and print+capture simulation puts the knee
        # at 200-250 um. Below ~200 um realistic silkscreen bleed starts killing it.
        m = args.qr_module
        rects, size = qr_rects(matrix, draw_light, quiet)
        span = size * m
        polys = [
            [(c0 * m - span / 2, r0 * m - span / 2), (c1 * m - span / 2, r0 * m - span / 2),
             (c1 * m - span / 2, r1 * m - span / 2), (c0 * m - span / 2, r1 * m - span / 2)]
            for c0, r0, c1, r1 in rects
        ]
        name = f"QR_HydroMix_{m*1000:.0f}um_{polarity}"
        descr = f"QR code: {args.url}"
        (outdir / f"{name}.kicad_mod").write_text(
            footprint(name, polys, ["F.SilkS"], descr, "qr logo", span / 2 + 0.8)
        )
        symbols.append(
            symbol(f"QR_HydroMix_{polarity}", polys, args.sym_size,
                   f"{nick}:{name}", descr)
        )
        print(f"qr {polarity}: v{qr.version}-{qr.error.upper()} {n}x{n} modules @ "
              f"{m*1000:.0f} um, {len(rects)} polygons, {span:.2f} x {span:.2f} mm "
              f"overall (silk quiet zone {quiet} modules)")
        if m < 0.20:
            print(f"      WARNING: {m*1000:.0f} um modules are below the 200 um knee "
                  f"-- silkscreen bleed will start breaking decodes")
        if span > args.qr_max:
            print(f"      WARNING: {span:.2f} mm exceeds --qr-max {args.qr_max:g} mm")
        if not draw_light:
            print(f"      keep {args.qr_quiet * m:.2f} mm of board clear around it "
                  f"-- the quiet zone is bare board, not silk")
        if polarity == "SilkOnDark":
            preview(prevdir / "preview_qr.svg",
                    [("#ffffff", [[(x + span / 2, y + span / 2) for x, y in p] for p in polys])],
                    span, span)

    print(f"previews: {prevdir}/preview_*.svg")

    Path(args.sym_lib).write_text(sym_lib(symbols))
    print(f"symbols: {len(symbols)} in {args.sym_lib} "
          f"(drawn {args.sym_size:g} mm on the sheet)")


if __name__ == "__main__":
    main()
