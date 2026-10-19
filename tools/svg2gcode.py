#!/usr/bin/env python3
"""SVG -> G-code for "scara gcode" (G0/G1 only, no arcs, no Z).

Every stroke of the drawing becomes: G0 to its start, M3, G1 segments, M5.
Curves are cut into short straight segments (--step mm). The drawing is
scaled so its longer side is --size mm and centred on --center X,Y (mm,
robot coordinates). Facing the arm with it pointing right (+X) and the
table (+Y) at the bottom, the drawing reads upright; --rotate turns it
counter-clockwise.

Text: in Inkscape use Extensions > Text > Hershey Text (single-line fonts);
other text must be turned into paths first (Path > Object to Path).

    python3 tools/svg2gcode.py drawing.svg -o drawing.gcode
    python3 tools/svg2gcode.py logo.svg --size 120 --center 210,-100 --feed 900

Python standard library only.
"""

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET

NUM = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")
SKIP = {"defs", "clipPath", "mask", "symbol", "metadata", "style", "title",
        "desc", "marker", "pattern", "linearGradient", "radialGradient",
        "filter", "namedview"}


def tag(el):
    return el.tag.rsplit("}", 1)[-1]


# --- affine transforms: (a, b, c, d, e, f) as in SVG matrix() -----------

IDENT = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def mul(m, n):
    a, b, c, d, e, f = m
    A, B, C, D, E, F = n
    return (a * A + c * B, b * A + d * B, a * C + c * D, b * C + d * D,
            a * E + c * F + e, b * E + d * F + f)


def apply(m, x, y):
    return m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]


def parse_transform(s):
    m = IDENT
    for name, args in re.findall(r"(\w+)\s*\(([^)]*)\)", s or ""):
        v = [float(x) for x in NUM.findall(args)]
        if name == "matrix" and len(v) == 6:
            t = tuple(v)
        elif name == "translate":
            t = (1, 0, 0, 1, v[0], v[1] if len(v) > 1 else 0)
        elif name == "scale":
            t = (v[0], 0, 0, v[1] if len(v) > 1 else v[0], 0, 0)
        elif name == "rotate":
            r = math.radians(v[0])
            t = (math.cos(r), math.sin(r), -math.sin(r), math.cos(r), 0, 0)
            if len(v) == 3:
                t = mul(mul((1, 0, 0, 1, v[1], v[2]), t),
                        (1, 0, 0, 1, -v[1], -v[2]))
        elif name == "skewX":
            t = (1, 0, math.tan(math.radians(v[0])), 1, 0, 0)
        elif name == "skewY":
            t = (1, math.tan(math.radians(v[0])), 0, 1, 0, 0)
        else:
            continue
        m = mul(m, t)
    return m


# --- path flattening ----------------------------------------------------

class Flattener:
    """Collects strokes (lists of points in drawing units, after transforms).

    seg: segment length in drawing units; 0 = coarse (for the bounds only).
    """

    def __init__(self, seg):
        self.seg = seg
        self.strokes = []

    def count(self, length):
        if self.seg <= 0:
            return 16
        return max(1, int(math.ceil(length / self.seg)))

    def path(self, d, m):
        toks = re.findall(r"[MmZzLlHhVvCcSsQqTtAa]|" + NUM.pattern, d or "")
        i, cmd = 0, None
        cx = cy = sx = sy = 0.0
        lc = None          # last control point, for S/T
        cur = []           # current stroke, in transformed coordinates

        def num():
            nonlocal i
            v = float(toks[i])
            i += 1
            return v

        def flag():
            # arc flags may be written without separators: "a1 1 0 01 2 3"
            nonlocal i
            t = toks[i]
            if t in ("0", "1"):
                i += 1
                return int(t)
            toks[i] = t[1:]
            return int(t[0])

        def emit(pts):
            for p in pts:
                cur.append(apply(m, *p))

        def finish():
            nonlocal cur
            if len(cur) > 1:
                self.strokes.append(cur)
            cur = []

        def tlen(pts):
            q = [apply(m, *p) for p in pts]
            return sum(math.dist(q[k], q[k + 1]) for k in range(len(q) - 1))

        def cubic(p0, p1, p2, p3):
            n = self.count(tlen([p0, p1, p2, p3]))
            out = []
            for k in range(1, n + 1):
                t = k / n
                u = 1 - t
                out.append((u ** 3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t ** 3 * p3[0],
                            u ** 3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t ** 3 * p3[1]))
            emit(out)

        def quad(p0, p1, p2):
            cubic(p0, (p0[0] + 2 / 3 * (p1[0] - p0[0]), p0[1] + 2 / 3 * (p1[1] - p0[1])),
                  (p2[0] + 2 / 3 * (p1[0] - p2[0]), p2[1] + 2 / 3 * (p1[1] - p2[1])), p2)

        def arc(x1, y1, rx, ry, phi, fa, fs, x2, y2):
            # SVG 1.1 appendix F.6: endpoint to centre parameterisation
            if rx == 0 or ry == 0 or (x1, y1) == (x2, y2):
                emit([(x2, y2)])
                return
            rx, ry = abs(rx), abs(ry)
            ph = math.radians(phi)
            cp, sp = math.cos(ph), math.sin(ph)
            dx, dy = (x1 - x2) / 2, (y1 - y2) / 2
            x1p, y1p = cp * dx + sp * dy, -sp * dx + cp * dy
            lam = x1p ** 2 / rx ** 2 + y1p ** 2 / ry ** 2
            if lam > 1:
                rx, ry = rx * math.sqrt(lam), ry * math.sqrt(lam)
            num_ = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p
            den = rx * rx * y1p * y1p + ry * ry * x1p * x1p
            co = math.sqrt(max(0.0, num_ / den))
            if fa == fs:
                co = -co
            cxp, cyp = co * rx * y1p / ry, -co * ry * x1p / rx
            ccx = cp * cxp - sp * cyp + (x1 + x2) / 2
            ccy = sp * cxp + cp * cyp + (y1 + y2) / 2
            a1 = math.atan2((y1p - cyp) / ry, (x1p - cxp) / rx)
            a2 = math.atan2((-y1p - cyp) / ry, (-x1p - cxp) / rx)
            da = a2 - a1
            if fs and da < 0:
                da += 2 * math.pi
            elif not fs and da > 0:
                da -= 2 * math.pi
            k = math.sqrt(abs(m[0] * m[3] - m[1] * m[2]))
            n = self.count(abs(da) * max(rx, ry) * k)
            out = []
            for j in range(1, n + 1):
                a = a1 + da * j / n
                ex, ey = rx * math.cos(a), ry * math.sin(a)
                out.append((cp * ex - sp * ey + ccx, sp * ex + cp * ey + ccy))
            emit(out)

        while i < len(toks):
            if toks[i].isalpha():
                cmd = toks[i]
                i += 1
            elif cmd is None:
                raise ValueError("path data does not start with a command")
            rel = cmd.islower()
            ox, oy = (cx, cy) if rel else (0.0, 0.0)
            c = cmd.upper()
            if c == "M":
                finish()
                cx, cy = ox + num(), oy + num()
                sx, sy = cx, cy
                emit([(cx, cy)])
                cmd = "l" if rel else "L"   # further pairs are lines
                lc = None
            elif c == "Z":
                if cur:
                    emit([(sx, sy)])
                finish()
                cx, cy = sx, sy
                emit([(cx, cy)])            # a following command starts here
                lc = None
            elif c in "LHV":
                if c == "L":
                    cx, cy = ox + num(), oy + num()
                elif c == "H":
                    cx = ox + num()
                else:
                    cy = oy + num()
                emit([(cx, cy)])
                lc = None
            elif c == "C":
                p1 = (ox + num(), oy + num())
                p2 = (ox + num(), oy + num())
                p3 = (ox + num(), oy + num())
                cubic((cx, cy), p1, p2, p3)
                lc, (cx, cy) = ("C", p2), p3
            elif c == "S":
                p1 = (2 * cx - lc[1][0], 2 * cy - lc[1][1]) if lc and lc[0] == "C" else (cx, cy)
                p2 = (ox + num(), oy + num())
                p3 = (ox + num(), oy + num())
                cubic((cx, cy), p1, p2, p3)
                lc, (cx, cy) = ("C", p2), p3
            elif c == "Q":
                p1 = (ox + num(), oy + num())
                p2 = (ox + num(), oy + num())
                quad((cx, cy), p1, p2)
                lc, (cx, cy) = ("Q", p1), p2
            elif c == "T":
                p1 = (2 * cx - lc[1][0], 2 * cy - lc[1][1]) if lc and lc[0] == "Q" else (cx, cy)
                p2 = (ox + num(), oy + num())
                quad((cx, cy), p1, p2)
                lc, (cx, cy) = ("Q", p1), p2
            elif c == "A":
                rx, ry, phi = num(), num(), num()
                fa, fs = flag(), flag()
                x2, y2 = ox + num(), oy + num()
                arc(cx, cy, rx, ry, phi, fa, fs, x2, y2)
                cx, cy = x2, y2
                lc = None
        finish()

    def ellipse(self, cx, cy, rx, ry, m):
        if rx > 0 and ry > 0:
            self.path(f"M{cx + rx},{cy} A{rx},{ry} 0 1 1 {cx - rx},{cy} "
                      f"A{rx},{ry} 0 1 1 {cx + rx},{cy}", m)


def walk(el, m, fl, warn):
    t = tag(el)
    if t in SKIP or el.get("display") == "none" or "display:none" in (el.get("style") or ""):
        return
    m = mul(m, parse_transform(el.get("transform")))
    def f(k):
        v = NUM.findall(el.get(k) or "")
        return float(v[0]) if v else 0.0

    if t == "path":
        fl.path(el.get("d"), m)
    elif t == "line":
        fl.path(f"M{f('x1')},{f('y1')} L{f('x2')},{f('y2')}", m)
    elif t in ("polyline", "polygon"):
        v = NUM.findall(el.get("points") or "")
        if len(v) >= 4:
            fl.path("M" + " ".join(v) + (" Z" if t == "polygon" else ""), m)
    elif t == "rect":
        x, y, w, h = f("x"), f("y"), f("width"), f("height")
        fl.path(f"M{x},{y} h{w} v{h} h{-w} Z", m)
    elif t == "circle":
        fl.ellipse(f("cx"), f("cy"), f("r"), f("r"), m)
    elif t == "ellipse":
        fl.ellipse(f("cx"), f("cy"), f("rx"), f("ry"), m)
    elif t in ("text", "flowRoot", "image", "use"):
        warn.add(t)
        return
    for ch in el:
        walk(ch, m, fl, warn)


def strokes_of(root, seg):
    fl, warn = Flattener(seg), set()
    walk(root, IDENT, fl, warn)
    return fl.strokes, warn


def order(strokes):
    """Nearest-neighbour order, reversing strokes when that is shorter."""
    left, out, pos = list(strokes), [], None
    while left:
        if pos is None:
            out.append(left.pop(0))
        else:
            best = min(range(len(left)), key=lambda k: min(
                math.dist(pos, left[k][0]), math.dist(pos, left[k][-1])))
            s = left.pop(best)
            if math.dist(pos, s[-1]) < math.dist(pos, s[0]):
                s = s[::-1]
            out.append(s)
        pos = out[-1][-1]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("svg")
    ap.add_argument("-o", "--output", help="G-code file (default: stdout)")
    ap.add_argument("--size", type=float, default=150,
                    help="longer side of the drawing, mm (default 150)")
    ap.add_argument("--center", default="200,-90",
                    help="centre of the drawing X,Y in mm (default 200,-90: clear of the table)")
    ap.add_argument("--rotate", type=float, default=0,
                    help="turn the drawing counter-clockwise, degrees")
    ap.add_argument("--feed", type=float, default=1200,
                    help="drawing speed, mm/min (default 1200 = 20 mm/s)")
    ap.add_argument("--step", type=float, default=1.0,
                    help="segment length on curves, mm (default 1)")
    a = ap.parse_args()
    cx, cy = (float(v) for v in a.center.split(","))

    root = ET.parse(a.svg).getroot()
    coarse, warn = strokes_of(root, 0)
    for w in sorted(warn):
        print(f"warning: <{w}> skipped (text: Extensions > Text > Hershey "
              f"Text, or Path > Object to Path)", file=sys.stderr)
    pts = [p for s in coarse for p in s]
    if not pts:
        sys.exit("error: nothing to draw in " + a.svg)
    x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
    y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
    span = max(x1 - x0, y1 - y0) or 1.0
    k = a.size / span                                 # mm per drawing unit
    strokes, _ = strokes_of(root, a.step / k)

    r = math.radians(a.rotate)
    cr, sr = math.cos(r), math.sin(r)
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2

    def robot(p):
        # u right, w up as you face the arm (table at the bottom); SVG y
        # points down, and so does robot +Y (towards the table)
        u, w = (p[0] - mx) * k, (my - p[1]) * k
        return cx + cr * u - sr * w, cy - (sr * u + cr * w)

    clean = []
    for s in strokes:
        q = [robot(p) for p in s]
        keep = [q[0]]
        for p in q[1:]:
            if math.dist(p, keep[-1]) >= 0.05:
                keep.append(p)
        if len(keep) > 1:
            clean.append(keep)
    clean = order(clean)

    out = [f"; {a.svg}: {len(clean)} strokes, {a.size:g} mm, centre "
           f"X{cx:g} Y{cy:g}, made by tools/svg2gcode.py",
           "G21 G90", "G28"]
    n = 0
    for s in clean:
        out.append("G0 X%.2f Y%.2f" % s[0])
        out.append("M3")
        for j, p in enumerate(s[1:]):
            out.append("G1 X%.2f Y%.2f" % p + (" F%g" % a.feed if j == 0 else ""))
            n += 1
        out.append("M5")
    out.append("M2")
    text = "\n".join(out) + "\n"
    if a.output:
        with open(a.output, "w") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)
    xs = [p[0] for s in clean for p in s]
    ys = [p[1] for s in clean for p in s]
    print(f"{len(clean)} strokes, {n} G1 lines, X {min(xs):.1f}..{max(xs):.1f}"
          f" Y {min(ys):.1f}..{max(ys):.1f} mm", file=sys.stderr)


if __name__ == "__main__":
    main()
