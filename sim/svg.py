"""Minimal SVG drawing helpers (no extra packages)."""

from __future__ import annotations

from pathlib import Path


def _esc(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


class Svg:
    def __init__(self, width: float, height: float, bg: str = "#f7f3ea") -> None:
        self.width = width
        self.height = height
        self.bg = bg
        self.els: list[str] = []

    def add(self, raw: str) -> None:
        self.els.append(raw)

    def line(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        stroke: str = "#1b1b1b",
        sw: float = 1.7,
        dash: str | None = None,
        cap: str = "round",
        opacity: float = 1.0,
    ) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{stroke}" stroke-width="{sw}" stroke-linecap="{cap}" '
            f'opacity="{opacity}"{dash_attr}/>'
        )

    def polyline(
        self,
        pts: list[tuple[float, float]],
        stroke: str = "#1b1b1b",
        sw: float = 1.7,
        fill: str = "none",
        cap: str = "round",
        join: str = "round",
        dash: str | None = None,
    ) -> None:
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<polyline points="{d}" fill="{fill}" stroke="{stroke}" '
            f'stroke-width="{sw}" stroke-linecap="{cap}" stroke-linejoin="{join}"{dash_attr}/>'
        )

    def polygon(
        self,
        pts: list[tuple[float, float]],
        fill: str,
        stroke: str = "#1b1b1b",
        sw: float = 1.4,
    ) -> None:
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        self.add(
            f'<polygon points="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
            f'stroke-linejoin="round"/>'
        )

    def circle(
        self,
        cx: float,
        cy: float,
        r: float,
        fill: str = "#1b1b1b",
        stroke: str = "none",
        sw: float = 1.0,
    ) -> None:
        self.add(
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}" '
            f'stroke="{stroke}" stroke-width="{sw}"/>'
        )

    def rect(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        fill: str = "none",
        stroke: str = "#1b1b1b",
        sw: float = 1.4,
        rx: float = 0.0,
        dash: str | None = None,
    ) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
            f'rx="{rx:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{dash_attr}/>'
        )

    def path(
        self,
        d: str,
        stroke: str = "#1b1b1b",
        sw: float = 1.7,
        fill: str = "none",
        cap: str = "round",
        join: str = "round",
    ) -> None:
        self.add(
            f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
            f'stroke-linecap="{cap}" stroke-linejoin="{join}"/>'
        )

    def text(
        self,
        x: float,
        y: float,
        s: str,
        size: float = 13,
        fill: str = "#1b1b1b",
        anchor: str = "start",
        weight: str = "normal",
        font: str = "Noto Sans Thai, Sarabun, Tahoma, DejaVu Sans, sans-serif",
        italic: bool = False,
    ) -> None:
        style = "italic" if italic else "normal"
        self.add(
            f'<text x="{x:.1f}" y="{y:.1f}" fill="{fill}" font-size="{size}" '
            f'font-family="{font}" font-weight="{weight}" font-style="{style}" '
            f'text-anchor="{anchor}">{_esc(s)}</text>'
        )

    def group_start(self, extra: str = "") -> None:
        self.add(f"<g {extra}>" if extra else "<g>")

    def group_end(self) -> None:
        self.add("</g>")

    def to_string(self) -> str:
        body = "\n".join(self.els)
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width:.0f}" '
            f'height="{self.height:.0f}" viewBox="0 0 {self.width:.0f} {self.height:.0f}">\n'
            f'<rect width="100%" height="100%" fill="{self.bg}"/>\n'
            f"{body}\n</svg>\n"
        )

    def save(self, path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.to_string(), encoding="utf-8")
        return path


def node(svg: Svg, x: float, y: float, r: float = 3.0, fill: str = "#1b1b1b") -> None:
    svg.circle(x, y, r, fill=fill)


def ground(svg: Svg, x: float, y: float, stroke: str = "#1b1b1b") -> None:
    svg.line(x, y, x, y + 8, stroke=stroke, sw=1.7)
    svg.line(x - 12, y + 8, x + 12, y + 8, stroke=stroke, sw=2.2)
    svg.line(x - 7, y + 13, x + 7, y + 13, stroke=stroke, sw=1.8)
    svg.line(x - 3, y + 18, x + 3, y + 18, stroke=stroke, sw=1.5)
