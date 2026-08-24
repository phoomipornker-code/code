"""Draw the 240 VAC forward-converter schematic as SVG."""

from __future__ import annotations

from pathlib import Path

from .svg import Svg, ground, node

INK = "#1b1b1b"
MUTED = "#6b6358"
HOT = "#9a2f22"
SEC = "#1e4d7b"
OK = "#2a6b48"
CORE = "#4a4338"
PAPER = "#fbf7ee"
FILL = "#ffffff"


def _res_h(svg: Svg, x1: float, x2: float, y: float, stroke: str = INK) -> None:
    xm = (x1 + x2) / 2
    zig_w, zig_h, n = 28, 8, 6
    zx0 = xm - zig_w / 2
    svg.line(x1, y, zx0, y, stroke=stroke)
    pts = [(zx0, y)]
    dx = zig_w / n
    for i in range(n):
        pts.append((zx0 + dx * (i + 0.5), y - zig_h if i % 2 == 0 else y + zig_h))
    pts.append((zx0 + zig_w, y))
    svg.polyline(pts, stroke=stroke, sw=1.7)
    svg.line(zx0 + zig_w, y, x2, y, stroke=stroke)


def _res_v(svg: Svg, x: float, y1: float, y2: float, stroke: str = INK) -> None:
    ym = (y1 + y2) / 2
    zig_h, zig_w, n = 28, 8, 6
    zy0 = ym - zig_h / 2
    svg.line(x, y1, x, zy0, stroke=stroke)
    pts = [(x, zy0)]
    dy = zig_h / n
    for i in range(n):
        pts.append((x - zig_w if i % 2 == 0 else x + zig_w, zy0 + dy * (i + 0.5)))
    pts.append((x, zy0 + zig_h))
    svg.polyline(pts, stroke=stroke, sw=1.7)
    svg.line(x, zy0 + zig_h, x, y2, stroke=stroke)


def _cap_v(svg: Svg, x: float, y1: float, y2: float, stroke: str = INK, polar: bool = True) -> None:
    ym = (y1 + y2) / 2
    svg.line(x, y1, x, ym - 8, stroke=stroke)
    svg.line(x - 14, ym - 8, x + 14, ym - 8, stroke=stroke, sw=2.4)
    svg.line(x - 14, ym + 8, x + 14, ym + 8, stroke=stroke, sw=2.4)
    svg.line(x, ym + 8, x, y2, stroke=stroke)
    if polar:
        svg.text(x + 16, ym - 4, "+", size=13, fill=HOT, weight="700")


def _diode_h(svg: Svg, x1: float, x2: float, y: float, stroke: str = INK, fill: str = "#f3ead8") -> None:
    """Anode at x1, cathode at x2 (points toward x2)."""
    xm = (x1 + x2) / 2.0
    s = 1.0 if x2 >= x1 else -1.0
    xa, xc = xm - 12 * s, xm + 8 * s
    svg.line(x1, y, xa, y, stroke=stroke)
    svg.polygon(
        [(xa, y - 10), (xc, y), (xa, y + 10)],
        fill=fill,
        stroke=stroke,
        sw=1.6,
    )
    svg.line(xc, y - 11, xc, y + 11, stroke=stroke, sw=2.2)
    svg.line(xc, y, x2, y, stroke=stroke)


def vertical_diode_triangle(
    x: float, y1: float, y2: float, cathode_at: str
) -> list[tuple[float, float]]:
    """Three points of the diode triangle. Tip is the cathode.

    y1 is the top wire, y2 the bottom. Freewheel D2 uses cathode_at='top'
    (cathode on the L/D1 node, anode on secondary GND).
    """
    ym = (y1 + y2) / 2.0
    if cathode_at == "top":
        return [(x, ym - 8), (x - 10, ym + 12), (x + 10, ym + 12)]
    if cathode_at == "bottom":
        return [(x, ym + 8), (x - 10, ym - 12), (x + 10, ym - 12)]
    raise ValueError(f"cathode_at must be 'top' or 'bottom', got {cathode_at!r}")


def _diode_v(
    svg: Svg,
    x: float,
    y1: float,
    y2: float,
    *,
    cathode_at: str,
    stroke: str = INK,
    fill: str = "#f3ead8",
) -> None:
    """Vertical diode. Triangle points at the cathode."""
    tri = vertical_diode_triangle(x, y1, y2, cathode_at)
    tip_y = tri[0][1]
    base_y = tri[1][1]
    svg.line(x, y1, x, min(tip_y, base_y), stroke=stroke)
    svg.line(x - 11, tip_y, x + 11, tip_y, stroke=stroke, sw=2.2)
    svg.polygon(tri, fill=fill, stroke=stroke, sw=1.6)
    svg.line(x, max(tip_y, base_y), x, y2, stroke=stroke)


def _inductor_h(svg: Svg, x1: float, x2: float, y: float, loops: int = 4, stroke: str = INK) -> None:
    span = 14 * loops
    xs = (x1 + x2) / 2 - span / 2
    svg.line(x1, y, xs, y, stroke=stroke)
    d = f"M {xs:.1f} {y:.1f}"
    for i in range(loops):
        x = xs + 14 * i
        d += f" c 0 -14 14 -14 14 0"
    svg.path(d, stroke=stroke, sw=1.8)
    svg.line(xs + span, y, x2, y, stroke=stroke)


def _coil_v(
    svg: Svg,
    x: float,
    y1: float,
    y2: float,
    loops: int = 5,
    side: str = "left",
    stroke: str = INK,
) -> None:
    h = y2 - y1
    dy = h / loops
    sign = -1 if side == "left" else 1
    d = f"M {x:.1f} {y1:.1f}"
    for i in range(loops):
        d += f" c {sign * 16:.1f} 0 {sign * 16:.1f} {dy:.1f} 0 {dy:.1f}"
    svg.path(d, stroke=stroke, sw=1.9)


def _mosfet(svg: Svg, x: float, y_d: float, y_s: float, x_g: float, stroke: str = INK) -> None:
    """N-MOSFET: drain up, source down, gate left of channel."""
    y_ch = (y_d + y_s) / 2
    svg.line(x, y_d, x, y_ch - 16, stroke=stroke, sw=1.8)
    svg.line(x, y_ch + 16, x, y_s, stroke=stroke, sw=1.8)
    svg.line(x - 10, y_ch - 18, x - 10, y_ch + 18, stroke=stroke, sw=2.4)  # channel
    svg.line(x - 10, y_ch - 12, x, y_ch - 12, stroke=stroke)  # drain tap
    svg.line(x - 10, y_ch + 12, x, y_ch + 12, stroke=stroke)  # source tap
    svg.polygon(
        [(x, y_ch + 12), (x - 5, y_ch + 6), (x + 5, y_ch + 6)],
        fill=stroke,
        stroke=stroke,
        sw=0.5,
    )
    svg.line(x - 16, y_ch - 20, x - 16, y_ch + 20, stroke=stroke, sw=1.8)  # gate plate
    svg.line(x_g, y_ch, x - 16, y_ch, stroke=stroke, sw=1.8)


def _dot(svg: Svg, x: float, y: float, color: str = INK) -> None:
    svg.circle(x, y, 4.2, fill=color)


def draw_power_schematic(path: Path) -> Path:
    W, H = 1680, 920
    svg = Svg(W, H, bg="#efe8d8")
    svg.rect(16, 14, W - 32, H - 28, fill=PAPER, stroke="#d4c7b0", sw=1.4, rx=16)

    svg.text(40, 48, "วงจรฟอร์เวิร์ดคอนเวอร์เตอร์", size=26, weight="700")
    svg.text(
        40,
        74,
        "Single-Switch Forward + ขดรีเซ็ต Nr  ·  AC 240 V → DC 58 V / 5 A  ·  ETD49  48:21:48  ·  67 kHz",
        size=15,
        fill=MUTED,
    )

    # region tints
    svg.rect(28, 96, 250, 700, fill="#f6efe0", stroke="#e4d8c2", sw=1, rx=12)
    svg.rect(290, 96, 430, 700, fill="#f8ece6", stroke="#ead3c8", sw=1, rx=12)
    svg.rect(732, 96, 160, 700, fill="#eeeae2", stroke="#ddd4c4", sw=1, rx=12)
    svg.rect(904, 96, 748, 700, fill="#eaf1f6", stroke="#cddce8", sw=1, rx=12)

    svg.text(40, 118, "1. อินพุต AC + เรกติไฟเออร์", size=13, fill=HOT, weight="700")
    svg.text(304, 118, "2. สวิตช์ปฐมภูมิ + RCD + Nr", size=13, fill=HOT, weight="700")
    svg.text(744, 118, "3. T1", size=13, fill=CORE, weight="700")
    svg.text(920, 118, "4. ทุติยภูมิ + LC (isolated)", size=13, fill=SEC, weight="700")

    # --- AC input ---
    svg.text(48, 175, "L", size=14, weight="700")
    svg.text(48, 455, "N", size=14, weight="700")
    svg.circle(70, 168, 7, fill="none", stroke=INK, sw=1.6)
    svg.circle(70, 448, 7, fill="none", stroke=INK, sw=1.6)
    svg.line(77, 168, 118, 168)
    # fuse
    svg.rect(118, 160, 44, 16, fill=FILL, stroke=INK, sw=1.5, rx=2)
    svg.line(124, 168, 156, 168, sw=1.3)
    svg.line(162, 168, 188, 168)
    svg.text(140, 154, "F1", size=12, anchor="middle", weight="700")
    svg.text(140, 198, "2.5 A T", size=11, fill=MUTED, anchor="middle")
    svg.line(77, 448, 188, 448)

    # bridge
    cx, cy, s = 234, 308, 52
    svg.polygon(
        [(cx, cy - s), (cx + s, cy), (cx, cy + s), (cx - s, cy)],
        fill="#fff8ef",
        stroke=INK,
        sw=1.7,
    )
    svg.line(188, 168, 188, cy - 4)
    svg.line(188, cy - 4, cx - s, cy)  # AC to left corner
    svg.line(188, 448, 188, cy + 4)
    svg.line(188, cy + 4, cx - s, cy)
    node(svg, 188, 168)
    node(svg, 188, 448)
    svg.text(234, 314, "BR1", size=13, anchor="middle", weight="700")
    svg.text(234, 332, "GBU606", size=11, fill=MUTED, anchor="middle")
    svg.text(cx, cy - s - 6, "+", size=14, fill=HOT, anchor="middle", weight="700")

    vin_y = 168
    gnd_y = 620
    svg.line(cx, cy - s, cx, vin_y)  # + up to rail? wait + is top of diamond which is cy-s=256
    # Actually I connected AC to left. Top of diamond is + at (cx, cy-s)=(234, 256)
    # Let me connect top to Vin rail at y=168
    svg.line(234, 256, 234, vin_y)
    svg.line(234, vin_y, 700, vin_y, stroke=HOT, sw=2.0)  # Vin+ rail
    svg.line(234, cy + s, 234, gnd_y)
    svg.line(180, gnd_y, 700, gnd_y, sw=2.0)
    ground(svg, 200, gnd_y)
    svg.text(214, gnd_y + 32, "GND ปฐมภูมิ", size=11, fill=MUTED)

    # Cin
    _cap_v(svg, 280, vin_y, gnd_y)
    node(svg, 280, vin_y)
    node(svg, 280, gnd_y)
    svg.text(298, 390, "Cin", size=13, weight="700")
    svg.text(298, 408, "220 µF", size=11, fill=MUTED)
    svg.text(298, 424, "450 V", size=11, fill=MUTED)

    svg.text(40, 500, "AC 240 V", size=13, weight="700")
    svg.text(40, 518, "50 Hz", size=11, fill=MUTED)

    # --- Q1 MOSFET (compact) ---
    qx, qyd = 500, 268
    qys = qyd + 88
    qg = 410
    np_x = 690
    _coil_v(svg, np_x, 188, 268, loops=5, side="left", stroke=HOT)
    _dot(svg, np_x - 22, 192, HOT)
    svg.line(np_x, vin_y, np_x, 188, stroke=HOT)
    svg.line(np_x, 268, qx, 268, stroke=HOT)
    _mosfet(svg, qx, qyd, qys, qg, stroke=INK)
    svg.line(qx, qys, qx, gnd_y)
    node(svg, qx, 268)
    node(svg, qx, gnd_y)
    node(svg, np_x, vin_y)
    svg.text(518, 380, "Q1", size=14, weight="700")
    svg.text(518, 398, "STW20N95K5", size=11, fill=MUTED)
    svg.text(518, 414, "950 V", size=11, fill=MUTED)
    svg.text(388, qyd + 40, "G", size=11, fill=MUTED)

    # PWM box under the primary ground rail
    svg.rect(300, 668, 140, 72, fill="#fff", stroke=INK, sw=1.5, rx=8)
    svg.text(370, 696, "PWM", size=13, anchor="middle", weight="700")
    svg.text(370, 714, "67 kHz", size=12, anchor="middle", fill=MUTED)
    svg.text(370, 730, "D ≤ 0.45", size=12, anchor="middle", fill=HOT)
    gy = (qyd + qys) / 2
    svg.line(370, 668, 370, gnd_y + 8)
    svg.path(f"M 370 {gnd_y + 8:.1f} A 8 8 0 0 1 370 {gnd_y - 8:.1f}")
    svg.line(370, gnd_y - 8, 370, gy)
    svg.line(370, gy, qg, gy)

    # Compact RCD at the drain node: Ds then Cs || Rs to source/GND
    svg.line(qx, 268, 548, 268)
    _diode_h(svg, 548, 598, 268, fill="#fde8d8")
    node(svg, 598, 268)
    svg.line(598, 268, 598, 292)
    svg.line(588, 292, 608, 292, sw=2.2)
    svg.line(588, 304, 608, 304, sw=2.2)
    svg.line(598, 304, 598, qys)
    _res_v(svg, 630, 292, qys)
    svg.line(598, 292, 630, 292)
    svg.line(598, qys, 630, qys)
    svg.line(598, qys, qx, qys)
    node(svg, qx, qys)
    svg.text(642, 330, "Rs 100 Ω / 10 W", size=11, fill=MUTED)
    svg.text(610, 284, "Cs 1 nF / 2 kV", size=11, fill=MUTED)
    svg.text(548, 254, "Ds UF4007", size=11, fill=MUTED)
    svg.text(548, 238, "RCD (ขนาน DS)", size=12, fill=HOT, weight="700")

    # Reset winding Nr below primary
    _coil_v(svg, np_x, 360, 500, loops=5, side="left", stroke=HOT)
    _dot(svg, np_x - 22, 492, HOT)  # dot at GND end (opposite polarity)
    svg.line(np_x, 500, np_x, 548)
    svg.line(np_x, 548, 430, 548)
    svg.line(430, 548, 430, gnd_y)
    node(svg, 430, gnd_y)
    # Dr from top of Nr to Vin+
    svg.line(np_x, 360, 400, 360)
    _diode_h(svg, 400, 340, 360, fill="#fde8d8")  # anode at Nr, cathode toward Vin
    # Wait: Dr cathode should go to Vin+. Current during reset: from GND through Nr (dot) up, out undotted top, through Dr to Vin+.
    # Diode: anode at winding top, cathode at Vin+. Current leftward if winding is on the right.
    # I drew diode from 400 to 340, which is leftward (cathode at 340). Then 340 to Vin rail.
    svg.line(340, 360, 340, vin_y)
    node(svg, 340, vin_y)
    svg.text(360, 344, "Dr", size=12, weight="700")
    svg.text(348, 380, "UF4007", size=11, fill=MUTED)
    svg.text(500, 348, "Nr 48 T", size=12, fill=HOT, weight="700")
    svg.text(500, 230, "Np 48 T", size=12, fill=HOT, weight="700")
    svg.text(430, 580, "รีเซ็ตฟลักซ์กลับบัส", size=11, fill=MUTED)

    # --- Transformer core ---
    svg.line(724, 170, 724, 560, stroke=CORE, sw=3.2)
    svg.line(732, 170, 732, 560, stroke=CORE, sw=3.2)
    svg.text(728, 590, "ETD49", size=12, anchor="middle", weight="700")
    svg.text(728, 606, "N87", size=11, fill=MUTED, anchor="middle")
    svg.text(728, 150, "T1", size=16, anchor="middle", weight="700")

    # isolation barrier
    svg.line(860, 130, 860, 780, stroke=SEC, sw=1.4, dash="7 6")
    svg.text(860, 800, "แยกกราวนด์", size=12, fill=SEC, anchor="middle")

    # Secondary coil
    ns_x = 770
    _coil_v(svg, ns_x, 200, 470, loops=7, side="right", stroke=SEC)
    _dot(svg, ns_x + 22, 208, SEC)
    svg.text(790, 188, "Ns 21 T", size=12, fill=SEC, weight="700")

    # D1 forward
    svg.line(ns_x, 200, ns_x, 176)
    svg.line(ns_x, 176, 920, 176, stroke=SEC, sw=1.8)
    _diode_h(svg, 920, 990, 176, stroke=SEC, fill="#dceaf5")
    svg.line(990, 176, 1080, 176, stroke=SEC, sw=1.8)
    node(svg, 1080, 176)
    svg.text(948, 160, "D1", size=13, fill=SEC, weight="700")
    svg.text(930, 198, "MUR860", size=11, fill=MUTED)

    # D2 freewheel: cathode on the L/D1 node, anode on secondary GND
    _diode_v(svg, 1080, 176, 470, cathode_at="top", stroke=SEC, fill="#dceaf5")
    svg.line(1080, 470, 1080, gnd_y, stroke=SEC)
    svg.text(1100, 318, "D2", size=13, fill=SEC, weight="700")
    svg.text(1100, 336, "freewheel", size=11, fill=MUTED)
    svg.text(1100, 352, "แคโทดที่ L", size=11, fill=OK)
    svg.text(1100, 368, "MUR860", size=11, fill=MUTED)

    # L and Co
    _inductor_h(svg, 1080, 1240, 176, loops=5, stroke=SEC)
    svg.line(1240, 176, 1480, 176, stroke=SEC, sw=2.0)
    node(svg, 1240, 176)
    svg.text(1148, 154, "L  520 µH", size=13, fill=SEC, weight="700")

    _cap_v(svg, 1320, 176, gnd_y, stroke=SEC)
    node(svg, 1320, 176)
    node(svg, 1320, gnd_y)
    svg.text(1340, 390, "Co", size=13, fill=SEC, weight="700")
    svg.text(1340, 408, "680 µF", size=11, fill=MUTED)
    svg.text(1340, 424, "≥ 80 V", size=11, fill=MUTED)

    # secondary return
    svg.line(ns_x, 470, ns_x, gnd_y, stroke=SEC)
    svg.line(ns_x, gnd_y, 1540, gnd_y, stroke=SEC, sw=2.0)
    ground(svg, 900, gnd_y, stroke=SEC)
    svg.text(914, gnd_y + 32, "GND ทุติยภูมิ (แยก)", size=11, fill=SEC)

    # output
    svg.circle(1480, 176, 8, fill="none", stroke=SEC, sw=1.8)
    svg.circle(1480, gnd_y, 8, fill="none", stroke=SEC, sw=1.8)
    svg.line(1480, 176, 1548, 176, stroke=SEC, sw=2)
    svg.line(1480, gnd_y, 1548, gnd_y, stroke=SEC, sw=2)
    svg.rect(1548, 150, 96, 80, fill="#e7f3ea", stroke=OK, sw=1.6, rx=8)
    svg.text(1596, 178, "58 V", size=18, fill=OK, anchor="middle", weight="700")
    svg.text(1596, 200, "5 A", size=16, fill=OK, anchor="middle", weight="700")
    svg.text(1596, 218, "290 W", size=12, fill=MUTED, anchor="middle")
    svg.text(1496, 164, "+", size=14, fill=OK, weight="700")
    svg.text(1496, gnd_y - 8, "−", size=16, fill=SEC, weight="700")

    # load resistor sketch
    _res_v(svg, 1420, 176, gnd_y, stroke=SEC)
    node(svg, 1420, 176)
    node(svg, 1420, gnd_y)
    svg.text(1434, 390, "R", size=13, fill=SEC, weight="700")
    svg.text(1434, 408, "11.6 Ω", size=11, fill=MUTED)

    # notes
    svg.rect(28, 812, 1624, 80, fill="#fff", stroke="#d4c7b0", sw=1, rx=10)
    svg.text(48, 838, "จุดทำงานหลัก", size=13, weight="700")
    svg.text(
        48,
        860,
        "Vo = Vin · (Ns/Np) · D    ·    Dmax = Np/(Np+Nr) = 0.50  (ใช้ ≤ 0.45)    ·    Vds,reset ≈ 2 Vin  เมื่อ Nr = Np    ·    Q1: STW20N95K5 + ฮีตซิงก์",
        size=13,
        fill="#3f3a33",
    )
    svg.text(
        48,
        880,
        "จุดสีบนขด = ขั้วจุด (polarity). Np กับ Ns จุดเดียวกัน → ส่งพลังงานตอน Q1 เปิด. Nr จุดกลับด้าน → รีเซ็ตฟลักซ์ตอน Q1 ปิดผ่าน Dr. D2 แคโทดอยู่ที่โหนด L (หันขึ้น).",
        size=13,
        fill="#3f3a33",
    )
    return svg.save(path)


def draw_on_off(path: Path) -> Path:
    W, H = 1600, 780
    svg = Svg(W, H, bg="#efe8d8")
    svg.rect(16, 14, W - 32, H - 28, fill=PAPER, stroke="#d4c7b0", sw=1.4, rx=16)
    svg.text(40, 50, "สองช่วงของวงจรฟอร์เวิร์ด", size=24, weight="700")
    svg.text(40, 74, "กระแสกำลัง (เส้นหนา) และกระแสมากเนไทซิ่ง/รีเซ็ต (เส้นประ)", size=14, fill=MUTED)

    def mini(x0: float, title: str, subtitle: str, on: bool) -> None:
        svg.rect(x0, 96, 740, 640, fill="#fffdf8", stroke="#d7ccba", sw=1.2, rx=14)
        svg.text(x0 + 24, 128, title, size=18, weight="700", fill=HOT if on else SEC)
        svg.text(x0 + 24, 150, subtitle, size=13, fill=MUTED)

        vin_y, gnd_y = 200, 620
        tx = x0 + 300
        # rails
        svg.line(x0 + 50, vin_y, tx, vin_y, stroke=HOT if on else INK, sw=2.2 if on else 1.4)
        svg.line(x0 + 50, gnd_y, tx + 80, gnd_y, stroke=INK, sw=1.6)
        svg.text(x0 + 58, vin_y - 10, "Vin+", size=12, fill=HOT, weight="700")
        ground(svg, x0 + 80, gnd_y)

        # transformer
        svg.line(tx + 24, 210, tx + 24, 560, stroke=CORE, sw=3)
        svg.line(tx + 32, 210, tx + 32, 560, stroke=CORE, sw=3)
        _coil_v(svg, tx, 220, 300, 4, "left", HOT if on else INK)
        _coil_v(svg, tx, 400, 520, 4, "left", INK if on else HOT)
        _coil_v(svg, tx + 56, 230, 500, 6, "right", SEC)
        _dot(svg, tx - 20, 226, HOT)
        _dot(svg, tx - 20, 512, HOT)
        _dot(svg, tx + 76, 238, SEC)

        # Q1 as a switch
        qx = x0 + 180
        svg.line(tx, 300, qx, 300, stroke=HOT if on else INK, sw=2 if on else 1.4)
        svg.line(qx, 300, qx, 430, stroke=HOT if on else INK, sw=2 if on else 1.4)
        if on:
            svg.line(qx, 430, qx, 470, stroke=HOT, sw=2.4)
            svg.line(qx - 12, 470, qx + 12, 458, stroke=HOT, sw=2.6)  # closed switch
        else:
            svg.line(qx - 14, 448, qx + 10, 430, stroke=INK, sw=2.2)  # open
            svg.line(qx, 430, qx, 444, stroke=INK)
            svg.line(qx, 468, qx, 490, stroke=INK)
        svg.line(qx, 490, qx, gnd_y, stroke=HOT if on else INK, sw=2 if on else 1.4)
        svg.text(qx + 16, 456, "Q1 " + ("ON" if on else "OFF"), size=13, weight="700")

        # Dr
        svg.line(tx, 400, x0 + 140, 400)
        _diode_h(svg, x0 + 140, x0 + 90, 400, fill="#fde8d8")
        svg.line(x0 + 90, 400, x0 + 90, vin_y, stroke=INK if on else HOT, sw=1.4 if on else 2.4)
        svg.text(x0 + 100, 384, "Dr", size=12, weight="700")
        if not on:
            # reset current arrow
            svg.polyline(
                [(tx - 8, 510), (tx - 8, 410), (x0 + 100, 410)],
                stroke=HOT,
                sw=2.4,
                dash="6 4",
            )

        # secondary D1 L load D2
        sx = tx + 56
        svg.line(sx, 230, sx, 210, stroke=SEC, sw=2 if on else 1.4)
        svg.line(sx, 210, x0 + 560, 210, stroke=SEC, sw=2 if on else 1.4)
        _diode_h(svg, x0 + 430, x0 + 490, 210, stroke=SEC, fill="#dceaf5")
        svg.text(x0 + 448, 194, "D1", size=12, fill=SEC, weight="700")
        _inductor_h(svg, x0 + 520, x0 + 620, 210, 4, SEC)
        svg.line(x0 + 620, 210, x0 + 680, 210, stroke=SEC, sw=2)
        svg.text(x0 + 690, 216, "Vo", size=14, fill=OK, weight="700")
        _cap_v(svg, x0 + 640, 210, gnd_y, stroke=SEC)
        _res_v(svg, x0 + 680, 210, gnd_y, stroke=SEC)
        _diode_v(svg, x0 + 520, 210, 500, cathode_at="top", stroke=SEC, fill="#dceaf5")
        svg.text(x0 + 534, 360, "D2", size=12, fill=SEC, weight="700")
        svg.text(x0 + 534, 376, "แคโทดบน", size=10, fill=OK)
        svg.line(sx, 500, sx, gnd_y, stroke=SEC)
        svg.line(sx, gnd_y, x0 + 700, gnd_y, stroke=SEC, sw=1.6)

        if on:
            svg.polyline(
                [(x0 + 60, vin_y + 8), (tx - 6, vin_y + 8), (tx - 6, 296), (qx + 8, 296), (qx + 8, gnd_y - 8)],
                stroke=HOT,
                sw=3.2,
            )
            svg.polyline(
                [(sx + 10, 236), (sx + 10, 218), (x0 + 518, 218), (x0 + 620, 218), (x0 + 680, 400)],
                stroke=SEC,
                sw=3.0,
            )
            svg.text(x0 + 24, 700, "พลังงานไหลผ่านหม้อแปลงไป D1 และ L   ·   D2 ตัด   ·   ฟลักซ์ในแกนเพิ่มขึ้น", size=13, fill=HOT)
        else:
            svg.polyline(
                [
                    (x0 + 520, 218),
                    (x0 + 680, 218),
                    (x0 + 680, gnd_y - 6),
                    (x0 + 520, gnd_y - 6),
                    (x0 + 520, 218),
                ],
                stroke=OK,
                sw=3.0,
            )
            svg.text(x0 + 24, 700, "D1 ตัด · D2 ฟรีวีลรักษากระแส L · Nr+Dr คืนพลังงานแมกเนไทซิ่งเข้าบัส Vin", size=13, fill=SEC)

    mini(28, "ช่วงที่ 1 — Q1 เปิด  (0 < t < DTs)", "ส่งพลังงานไปทุติยภูมิ", True)
    mini(820, "ช่วงที่ 2 — Q1 ปิด  (DTs < t < Ts)", "ฟรีวีล + รีเซ็ตฟลักซ์", False)
    return svg.save(path)


def main() -> None:
    root = Path(__file__).resolve().parents[1] / "artifacts"
    p1 = draw_power_schematic(root / "forward_schematic.svg")
    p2 = draw_on_off(root / "forward_on_off.svg")
    print(p1)
    print(p2)


if __name__ == "__main__":
    main()
