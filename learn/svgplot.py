"""Tiny SVG line-chart helper so the lesson has no extra Python packages."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Series:
    name: str
    t: list[float]
    y: list[float]
    color: str
    width: float = 1.6


@dataclass
class HLine:
    y: float
    color: str
    dash: str = "6 4"
    label: str = ""


@dataclass
class Span:
    t0: float
    t1: float
    color: str
    label: str = ""


@dataclass
class Panel:
    title: str
    ylabel: str
    series: list[Series]
    hlines: list[HLine] = field(default_factory=list)
    y_min: float | None = None
    y_max: float | None = None


def _nice_range(lo: float, hi: float) -> tuple[float, float]:
    if hi <= lo:
        pad = 1.0 if lo == 0 else abs(lo) * 0.1
        return lo - pad, hi + pad
    pad = (hi - lo) * 0.08
    return lo - pad, hi + pad


def _ticks(lo: float, hi: float, n: int = 5) -> list[float]:
    if hi <= lo:
        return [lo]
    step = (hi - lo) / (n - 1)
    return [lo + i * step for i in range(n)]


def write_stack(
    path: str,
    panels: list[Panel],
    spans: list[Span],
    xlabel: str,
    title: str,
    t_max: float,
) -> None:
    width = 920
    left, right, top, gap, bottom = 70, 24, 48, 18, 48
    panel_h = 150
    height = top + bottom + len(panels) * panel_h + (len(panels) - 1) * gap
    plot_w = width - left - right

    def x_of(t: float) -> float:
        return left + (t / t_max) * plot_w if t_max > 0 else left

    parts: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="DejaVu Sans, Arial, sans-serif">',
        '<rect width="100%" height="100%" fill="#fbfbfd"/>',
        f'<text x="{width / 2:.1f}" y="28" text-anchor="middle" font-size="16" '
        f'font-weight="700" fill="#1a1a1a">{_esc(title)}</text>',
    ]

    legend_x = left
    for span in _unique_spans(spans):
        parts.append(
            f'<rect x="{legend_x:.1f}" y="8" width="12" height="12" fill="{span.color}" opacity="0.45"/>'
        )
        parts.append(
            f'<text x="{legend_x + 16:.1f}" y="18" font-size="11" fill="#333">{_esc(span.label)}</text>'
        )
        legend_x += 12 + 16 + len(span.label) * 6.2 + 16

    for i, panel in enumerate(panels):
        y0 = top + i * (panel_h + gap)
        ys = [v for s in panel.series for v in s.y]
        ys += [h.y for h in panel.hlines]
        data_lo = min(ys) if ys else 0.0
        data_hi = max(ys) if ys else 1.0
        y_lo = panel.y_min if panel.y_min is not None else _nice_range(data_lo, data_hi)[0]
        y_hi = panel.y_max if panel.y_max is not None else _nice_range(data_lo, data_hi)[1]
        if y_hi <= y_lo:
            y_hi = y_lo + 1.0

        def y_of(v: float, _y0: float = y0, _lo: float = y_lo, _hi: float = y_hi) -> float:
            return _y0 + panel_h - ((v - _lo) / (_hi - _lo)) * panel_h

        parts.append(
            f'<rect x="{left}" y="{y0}" width="{plot_w}" height="{panel_h}" '
            f'fill="#ffffff" stroke="#d0d4da"/>'
        )
        for span in spans:
            x1 = x_of(span.t0)
            x2 = x_of(span.t1)
            parts.append(
                f'<rect x="{x1:.2f}" y="{y0}" width="{max(0.0, x2 - x1):.2f}" '
                f'height="{panel_h}" fill="{span.color}" opacity="0.16"/>'
            )

        for tick in _ticks(y_lo, y_hi):
            yy = y_of(tick)
            parts.append(
                f'<line x1="{left}" y1="{yy:.2f}" x2="{left + plot_w}" y2="{yy:.2f}" '
                f'stroke="#eef0f3"/>'
            )
            parts.append(
                f'<text x="{left - 8}" y="{yy + 3:.2f}" text-anchor="end" font-size="10" '
                f'fill="#555">{_fmt(tick)}</text>'
            )

        for hline in panel.hlines:
            yy = y_of(hline.y)
            parts.append(
                f'<line x1="{left}" y1="{yy:.2f}" x2="{left + plot_w}" y2="{yy:.2f}" '
                f'stroke="{hline.color}" stroke-dasharray="{hline.dash}" stroke-width="1"/>'
            )
            if hline.label:
                parts.append(
                    f'<text x="{left + plot_w - 6}" y="{yy - 4:.2f}" text-anchor="end" '
                    f'font-size="10" fill="{hline.color}">{_esc(hline.label)}</text>'
                )

        for series in panel.series:
            if len(series.t) < 2:
                continue
            pts = " ".join(
                f"{x_of(t):.2f},{y_of(v):.2f}" for t, v in zip(series.t, series.y)
            )
            parts.append(
                f'<polyline fill="none" stroke="{series.color}" stroke-width="{series.width}" '
                f'stroke-linejoin="round" points="{pts}"/>'
            )

        lx = left + 8
        ly = y0 + 14
        parts.append(
            f'<text x="{lx}" y="{ly}" font-size="12" font-weight="700" fill="#222">'
            f'{_esc(panel.title)}</text>'
        )
        lx += 8 + len(panel.title) * 7
        for series in panel.series:
            parts.append(
                f'<line x1="{lx}" y1="{ly - 4}" x2="{lx + 16}" y2="{ly - 4}" '
                f'stroke="{series.color}" stroke-width="2"/>'
            )
            parts.append(
                f'<text x="{lx + 20}" y="{ly}" font-size="11" fill="#333">{_esc(series.name)}</text>'
            )
            lx += 28 + len(series.name) * 6.4

        parts.append(
            f'<text x="16" y="{y0 + panel_h / 2:.1f}" font-size="11" fill="#444" '
            f'transform="rotate(-90 16 {y0 + panel_h / 2:.1f})">{_esc(panel.ylabel)}</text>'
        )

        if i == len(panels) - 1:
            for tick in _ticks(0.0, t_max, 7):
                xx = x_of(tick)
                parts.append(
                    f'<text x="{xx:.2f}" y="{y0 + panel_h + 16:.2f}" text-anchor="middle" '
                    f'font-size="10" fill="#555">{_fmt(tick)}</text>'
                )
            parts.append(
                f'<text x="{left + plot_w / 2:.1f}" y="{y0 + panel_h + 34:.2f}" '
                f'text-anchor="middle" font-size="12" fill="#333">{_esc(xlabel)}</text>'
            )

    parts.append("</svg>")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(parts))
        fh.write("\n")


def _unique_spans(spans: list[Span]) -> list[Span]:
    seen: dict[str, Span] = {}
    for span in spans:
        if span.label and span.label not in seen:
            seen[span.label] = span
    return list(seen.values())


def _fmt(v: float) -> str:
    if abs(v) >= 100:
        return f"{v:.0f}"
    if abs(v) >= 10:
        return f"{v:.1f}"
    return f"{v:.2f}"


def _esc(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )
