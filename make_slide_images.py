#!/usr/bin/env python3
"""
Builds the result images used by navyseals_presentation.html.

  slides_assets/sprint_satellite.png   - Dock -> A -> B -> A sprint on satellite imagery
  slides_assets/sprint_satellite_wide.png - same, wider crop for the A4 report
  slides_assets/box_satellite.png      - 15 m box waypoint mission on satellite imagery
  slides_assets/sprint_telemetry.png   - distance-to-target and motor commands over time

Inputs:
  NavySeals/mission_log.csv            - sprint run (current working copy)
  NavySeals/box_mission_log.csv        - 15 m box run (copy of the log from commit e5b681a)

Satellite tiles come from Esri World Imagery and are cached in slides_assets/.tile_cache.
Run:  python make_slide_images.py
"""

import argparse
import csv
import io
import math
import os
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import numpy as np
import requests
from PIL import Image

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "slides_assets")
CACHE = os.path.join(OUT, ".tile_cache")

# Local frame used by waypoint_mission.py (Guerledan reference)
LAT_REF, LON_REF = 48.199111, -3.014930
R_EARTH = 6371000.0
POINT_A = (48.198863, -3.015070)
POINT_B = (48.198259, -3.017114)

TILE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
ATTRIBUTION = "Imagery: Esri, Maxar, Earthstar Geographics"

FONT = "DejaVu Sans"
plt.rcParams.update({"font.family": FONT})


# ---------------------------------------------------------------- geometry
def xy_to_latlon(x, y):
    lat = LAT_REF + math.degrees(y / R_EARTH)
    lon = LON_REF + math.degrees(x / (R_EARTH * math.cos(math.radians(LAT_REF))))
    return lat, lon


def latlon_to_world_px(lat, lon, z):
    """Web-Mercator world pixel coordinates at zoom z (256 px tiles)."""
    n = 256 * 2 ** z
    px = (lon + 180.0) / 360.0 * n
    s = math.sin(math.radians(lat))
    py = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n
    return px, py


def metres_per_px(lat, z):
    return 156543.03392 * math.cos(math.radians(lat)) / 2 ** z


# ---------------------------------------------------------------- tiles
def get_tile(z, x, y):
    os.makedirs(CACHE, exist_ok=True)
    path = os.path.join(CACHE, "{}_{}_{}.jpg".format(z, x, y))
    if not os.path.exists(path):
        r = requests.get(TILE_URL.format(z=z, x=x, y=y), timeout=20,
                         headers={"User-Agent": "NavySeals-slides/1.0"})
        r.raise_for_status()
        with open(path, "wb") as f:
            f.write(r.content)
    return Image.open(path).convert("RGB")


def satellite_canvas(lat_min, lat_max, lon_min, lon_max, z, aspect=16 / 9, pad_frac=0.18,
                     shift=(0.0, 0.0)):
    """Stitch tiles covering the box (padded, widened to `aspect`). Returns image and px offset.
    `shift` moves the frame centre by a fraction of its width/height (to make room for labels)."""
    x0, y1 = latlon_to_world_px(lat_min, lon_min, z)
    x1, y0 = latlon_to_world_px(lat_max, lon_max, z)
    w, h = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    w, h = w * (1 + 2 * pad_frac), h * (1 + 2 * pad_frac)
    if w / h < aspect:
        w = h * aspect
    else:
        h = w / aspect
    cx, cy = cx + shift[0] * w, cy + shift[1] * h
    left, top = int(cx - w / 2), int(cy - h / 2)
    right, bottom = int(cx + w / 2), int(cy + h / 2)

    tx0, ty0 = left // 256, top // 256
    tx1, ty1 = right // 256, bottom // 256
    big = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            big.paste(get_tile(z, tx, ty), ((tx - tx0) * 256, (ty - ty0) * 256))
    crop = big.crop((left - tx0 * 256, top - ty0 * 256, right - tx0 * 256, bottom - ty0 * 256))
    return crop, left, top


# ---------------------------------------------------------------- logs
def read_log(text):
    rows = list(csv.DictReader(io.StringIO(text)))
    for r in rows:
        for k in ("time", "x", "y", "target_x", "target_y", "heading_deg", "error_deg"):
            r[k] = float(r[k])
        r["wp_idx"] = int(r["wp_idx"])
        r["u1"], r["u2"] = int(r["u1"]), int(r["u2"])
    return rows


def unique_track(rows):
    """Drop repeated GPS positions (log runs at ~10 Hz, GPS at ~1 Hz)."""
    out, prev = [], None
    for r in rows:
        p = (r["x"], r["y"])
        if p != prev:
            out.append(r)
        prev = p
    return out


# ---------------------------------------------------------------- drawing helpers
GLOW = [pe.Stroke(linewidth=9, foreground=(0, 0, 0, 0.45)), pe.Normal()]
TXT_SHADOW = [pe.withStroke(linewidth=4, foreground=(0, 0, 0, 0.75))]


def pin(ax, px, py, color, label, dx=18, dy=-18, ha="left"):
    ax.plot(px, py, marker="o", ms=22, mfc=color, mec="white", mew=3.5, zorder=6)
    ax.annotate(label, (px, py), xytext=(dx, dy), textcoords="offset points",
                ha=ha, va="center", fontsize=24, fontweight="bold", color="#0f172a", zorder=7,
                bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="none", alpha=0.95))


def new_map_figure(img, out_w):
    """Figure whose pixel size is out_w wide with the image's aspect; axes use tile-pixel coords."""
    w, h = img.size
    out_h = int(round(out_w * h / w))
    fig = plt.figure(figsize=(out_w / 100, out_h / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img, extent=(0, w, h, 0), zorder=0, interpolation="lanczos")
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")
    return fig, ax


def scale_bar(ax, m_per_px, metres, loc="left"):
    """Compact scale bar at the bottom-left, drawn in data (tile px) coords."""
    x_lo, x_hi = ax.get_xlim()
    y_hi, y_lo = ax.get_ylim()  # inverted y
    W, H = x_hi - x_lo, y_hi - y_lo
    px_len = metres / m_per_px
    x0 = x_lo + W * 0.03 if loc == "left" else x_hi - W * 0.03 - px_len
    y0 = y_lo + H * (0.945 if loc == "left" else 0.92)
    ax.plot([x0, x0 + px_len], [y0, y0], color="white", lw=5, solid_capstyle="butt", zorder=9,
            path_effects=[pe.withStroke(linewidth=8, foreground="black")])
    for xx in (x0, x0 + px_len):
        ax.plot([xx, xx], [y0 - H * 0.012, y0 + H * 0.012], color="white", lw=3, zorder=9,
                path_effects=[pe.withStroke(linewidth=6, foreground="black")])
    ax.text(x0 + px_len / 2, y0 - H * 0.028, "{:d} m".format(int(metres)), ha="center", va="center",
            fontsize=23, fontweight="bold", color="white", zorder=9, path_effects=TXT_SHADOW)


def north_arrow(ax, xy=(0.965, 0.90)):
    x, y = xy
    ax.annotate("", xy=(x, y + 0.045), xytext=(x, y - 0.035), xycoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>,head_width=0.55,head_length=0.9", color="white", lw=3.5,
                                path_effects=[pe.withStroke(linewidth=6, foreground="black")]),
                zorder=9)
    ax.text(x, y - 0.05, "N", transform=ax.transAxes, ha="center", va="top", fontsize=23,
            fontweight="bold", color="white", zorder=9, path_effects=TXT_SHADOW)


def finish_map(fig, ax, m_per_px, bar_m, path, north_xy=(0.965, 0.90), bar_loc="left"):
    scale_bar(ax, m_per_px, bar_m, bar_loc)
    north_arrow(ax, north_xy)
    ax.text(0.99, 0.014, ATTRIBUTION, transform=ax.transAxes, ha="right", va="bottom", fontsize=14,
            color="white", alpha=0.9, zorder=9, path_effects=[pe.withStroke(linewidth=2.5, foreground="black")])
    fig.savefig(path, dpi=100)
    plt.close(fig)
    print("[OK] wrote", os.path.relpath(path, ROOT))


# ---------------------------------------------------------------- figures
LEG_COLORS = {0: "#22d3ee", 1: "#facc15", 2: "#fb923c"}
LEG_NAMES = {0: "Dock to A", 1: "A to B", 2: "B back to A"}


def sprint_map(rows, path, z=19, aspect=1.69, out_w=1920, clean=False):
    track = unique_track(rows)
    lls = [xy_to_latlon(r["x"], r["y"]) for r in track]
    lats = [p[0] for p in lls] + [POINT_A[0], POINT_B[0]]
    lons = [p[1] for p in lls] + [POINT_A[1], POINT_B[1]]
    img, left, top = satellite_canvas(min(lats), max(lats), min(lons), max(lons), z,
                                      aspect=aspect, pad_frac=0.2, shift=(0.0, 0.03))

    def to_px(lat, lon):
        X, Y = latlon_to_world_px(lat, lon, z)
        return X - left, Y - top

    fig, ax = new_map_figure(img, out_w)

    # planned course line A <-> B
    ax_, ay_ = to_px(*POINT_A)
    bx_, by_ = to_px(*POINT_B)
    ax.plot([ax_, bx_], [ay_, by_], color="white", lw=2.5, ls=(0, (6, 6)), alpha=0.85, zorder=2)

    # track coloured by leg (include the first point of the next leg so segments join)
    for leg in (0, 1, 2):
        idx = [i for i, r in enumerate(track) if r["wp_idx"] == leg]
        if not idx:
            continue
        idx = idx + ([idx[-1] + 1] if idx[-1] + 1 < len(track) else [])
        pts = [to_px(*lls[i]) for i in idx]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=LEG_COLORS[leg], lw=5,
                solid_capstyle="round", path_effects=GLOW, zorder=4, label=LEG_NAMES[leg])

    sx, sy = to_px(*lls[0])
    ex, ey = to_px(*lls[-1])
    if clean:  # background version: track and buoys only
        for px, py, c in ((ax_, ay_, "#16a34a"), (bx_, by_, "#dc2626")):
            ax.plot(px, py, marker="o", ms=20, mfc=c, mec="white", mew=3.5, zorder=6)
        ax.text(0.99, 0.014, ATTRIBUTION, transform=ax.transAxes, ha="right", va="bottom", fontsize=14,
                color="white", alpha=0.9, zorder=9, path_effects=[pe.withStroke(linewidth=2.5, foreground="black")])
        fig.savefig(path, dpi=100)
        plt.close(fig)
        print("[OK] wrote", os.path.relpath(path, ROOT))
        return
    ax.plot(sx, sy, marker="s", ms=17, mfc="#a855f7", mec="white", mew=3, zorder=6)
    ax.annotate("Launch", (sx, sy), xytext=(0, 32), textcoords="offset points", ha="center",
                fontsize=23, fontweight="bold", color="white", path_effects=TXT_SHADOW, zorder=7)
    ax.plot(ex, ey, marker="D", ms=15, mfc="white", mec="#fb923c", mew=3.5, zorder=6)
    ax.annotate("Wi-Fi lost here", (ex, ey), xytext=(20, -34), textcoords="offset points", ha="left",
                fontsize=23, fontweight="bold", color="white", path_effects=TXT_SHADOW, zorder=7)

    pin(ax, ax_, ay_, "#16a34a", "Point A · shore buoy", dx=-10, dy=-46, ha="right")
    pin(ax, bx_, by_, "#dc2626", "Point B · lake buoy", dx=10, dy=-46, ha="left")

    leg = ax.legend(loc="upper left", fontsize=22, frameon=True, framealpha=0.93, borderpad=0.9,
                    handlelength=2.2, title="Sprint run · 380 GPS fixes", title_fontsize=22,
                    bbox_to_anchor=(0.02, 0.975))
    leg.get_frame().set_edgecolor("none")
    for t in leg.get_texts():
        t.set_color("#0f172a")
    finish_map(fig, ax, metres_per_px(POINT_A[0], z), 50, path, bar_loc="right" if aspect > 2 else "left")


def box_map(rows, path, z=19):
    track = unique_track(rows)
    lls = [xy_to_latlon(r["x"], r["y"]) for r in track]
    targets = []
    for r in rows:
        t = (r["target_x"], r["target_y"])
        if not targets or targets[-1] != t:
            targets.append(t)
    tll = [xy_to_latlon(*t) for t in targets]
    lats = [p[0] for p in lls + tll]
    lons = [p[1] for p in lls + tll]
    # wide frame (~CONTEXT_M across) so the shoreline and docks are visible
    CONTEXT_M = 150.0
    box_w_m = (max(lons) - min(lons)) * math.radians(1) * R_EARTH * math.cos(math.radians(LAT_REF))
    pad = max(0.0, (CONTEXT_M / max(box_w_m, 1.0) - 1) / 2)
    img, left, top = satellite_canvas(min(lats), max(lats), min(lons), max(lons), z,
                                      aspect=4 / 3, pad_frac=pad, shift=(0.21, 0.0))

    def to_px(lat, lon):
        X, Y = latlon_to_world_px(lat, lon, z)
        return X - left, Y - top

    fig, ax = new_map_figure(img, 1440)
    mpp = metres_per_px(lls[0][0], z)
    pts = [to_px(*p) for p in lls]
    tpx = [to_px(*p) for p in tll]

    # main (context) view: track + a frame showing the zoomed area
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#facc15", lw=3.5,
            solid_capstyle="round", path_effects=[pe.Stroke(linewidth=6, foreground=(0, 0, 0, 0.5)), pe.Normal()],
            zorder=4)
    xs = [p[0] for p in pts + tpx]
    ys = [p[1] for p in pts + tpx]
    m = 6.0 / mpp  # 6 m margin around the box
    zx0, zx1, zy0, zy1 = min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m
    ax.add_patch(plt.Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, fc="none", ec="white", lw=2.5,
                               ls=(0, (6, 4)), zorder=5))

    # zoomed inset (right side)
    ins = fig.add_axes([0.575, 0.27, 0.40, 0.53])
    ins.imshow(img, extent=(0, img.size[0], img.size[1], 0), interpolation="bicubic", zorder=0)
    zw, zh = zx1 - zx0, zy1 - zy0
    cxz, cyz = (zx0 + zx1) / 2, (zy0 + zy1) / 2
    ins_w = max(zw, zh * (0.40 * 1440) / (0.53 * 1080))
    ins_h = ins_w * (0.53 * 1080) / (0.40 * 1440)
    ins.set_xlim(cxz - ins_w / 2, cxz + ins_w / 2)
    ins.set_ylim(cyz + ins_h / 2, cyz - ins_h / 2)
    ins.set_xticks([])
    ins.set_yticks([])
    for s in ins.spines.values():
        s.set_edgecolor("white")
        s.set_linewidth(4)
    for (px, py) in tpx[:4]:
        ins.add_patch(plt.Circle((px, py), 2.5 / mpp, fc=(1, 1, 1, 0.14), ec="white", lw=2,
                                 ls=(0, (5, 4)), zorder=3))
    ins.plot([p[0] for p in pts], [p[1] for p in pts], color="#facc15", lw=5,
             solid_capstyle="round", path_effects=GLOW, zorder=4)
    labels = ["WP1", "WP2", "WP3", "WP4 · home"]
    offs = [(18, -30, "left"), (18, 30, "left"), (-18, 30, "right"), (0, -40, "center")]
    for i, (px, py) in enumerate(tpx[:4]):
        dx, dy, ha = offs[i]
        ins.plot(px, py, marker="o", ms=13, mfc="#dc2626", mec="white", mew=3, zorder=6)
        ins.annotate(labels[i], (px, py), xytext=(dx, dy), textcoords="offset points", ha=ha,
                     va="center", fontsize=21, fontweight="bold", color="#0f172a", zorder=7,
                     bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="none", alpha=0.95))
    sx, sy = pts[0]
    ins.plot(sx, sy, marker="s", ms=14, mfc="#a855f7", mec="white", mew=3, zorder=6)
    # inset scale bar (5 m)
    bx0 = cxz - ins_w / 2 + ins_w * 0.05
    by0 = cyz - ins_h / 2 + ins_h * 0.13
    ins.plot([bx0, bx0 + 5 / mpp], [by0, by0], color="white", lw=4, zorder=9,
             path_effects=[pe.withStroke(linewidth=7, foreground="black")])
    ins.text(bx0 + 2.5 / mpp, by0 - ins_h * 0.045, "5 m", ha="center", fontsize=20, fontweight="bold",
             color="white", path_effects=TXT_SHADOW, zorder=9)
    ins.text(0.5, 0.965, "Zoom · 15 m box", transform=ins.transAxes, ha="center", va="top", fontsize=21,
             fontweight="bold", color="#0f172a", zorder=9,
             bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="none", alpha=0.95))
    # connector lines from frame to inset
    fig_to_data = ax.transData.inverted()
    for (fx, fy), (cx_, cy_) in (((0.575, 0.80), (zx1, zy0)), ((0.575, 0.27), (zx1, zy1))):
        dx_, dy_ = fig_to_data.transform(fig.transFigure.transform((fx, fy)))
        ax.plot([cx_, dx_], [cy_, dy_], color="white", lw=1.8, alpha=0.8, zorder=5)
    finish_map(fig, ax, mpp, 20, path, north_xy=(0.05, 0.86))


def telemetry_chart(rows, path):
    t0 = rows[0]["time"]
    t = np.array([r["time"] - t0 for r in rows])
    dist = np.array([math.hypot(r["target_x"] - r["x"], r["target_y"] - r["y"]) for r in rows])
    u1 = np.array([r["u1"] for r in rows])
    u2 = np.array([r["u2"] for r in rows])
    leg = np.array([r["wp_idx"] for r in rows])

    ink, muted, grid = "#0f172a", "#475569", "#e2e8f0"
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(15, 8.7), dpi=100, sharex=True,
                                 gridspec_kw=dict(height_ratios=[1.35, 1], hspace=0.12))
    fig.patch.set_facecolor("white")
    for ax in (a1, a2):
        ax.set_facecolor("white")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color("#cbd5e1")
        ax.tick_params(colors=muted, labelsize=20)
        ax.grid(True, color=grid, lw=1)
        ax.set_axisbelow(True)
        for lg in (0, 1, 2):
            m = np.where(leg == lg)[0]
            if len(m):
                ax.axvspan(t[m[0]], t[m[-1]], color=LEG_COLORS[lg], alpha=0.10, lw=0)

    for lg in (0, 1, 2):
        m = leg == lg
        a1.plot(t[m], dist[m], color={0: "#0891b2", 1: "#ca8a04", 2: "#ea580c"}[lg], lw=3)
    a1.set_ylabel("Distance to\ntarget (m)", fontsize=21, color=ink)
    a1.set_ylim(0, max(dist) * 1.12)
    ymax = a1.get_ylim()[1]
    for lg, name in ((1, "A → B"), (2, "B → A")):
        m = np.where(leg == lg)[0]
        a1.text((t[m[0]] + t[m[-1]]) / 2, ymax * 0.95, name, ha="center", va="top",
                fontsize=22, fontweight="bold", color=ink)
    m0 = np.where(leg == 0)[0]
    a1.text(t[m0[-1]] + 3, 22, "Dock → A", ha="left", va="center", fontsize=19, color="#0891b2",
            fontweight="bold")
    ib = np.where(leg == 1)[0]
    k = ib[np.argmin(dist[ib])]
    a1.annotate("Reached B\n({:.1f} m)".format(dist[k]), (t[k], dist[k]), xytext=(-40, 70),
                textcoords="offset points", ha="right", fontsize=20, color=ink,
                arrowprops=dict(arrowstyle="->", color=muted, lw=1.6))

    a2.plot(t, u1, color="#2563eb", lw=1.6, label="Left motor u1")
    a2.plot(t, u2, color="#e11d48", lw=1.6, alpha=0.85, label="Right motor u2")
    a2.axhline(0, color="#94a3b8", lw=1)
    a2.set_ylabel("Motor PWM", fontsize=21, color=ink)
    a2.set_xlabel("Mission time (s)", fontsize=21, color=ink)
    a2.set_ylim(-125, 125)
    lg = a2.legend(loc="lower right", fontsize=19, frameon=True, ncol=2, framealpha=0.95)
    lg.get_frame().set_edgecolor("none")
    a2.set_xlim(0, t[-1])

    fig.savefig(path, dpi=100, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("[OK] wrote", os.path.relpath(path, ROOT))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sprint-log", default=os.path.join(ROOT, "NavySeals", "mission_log.csv"))
    ap.add_argument("--box-log", default=os.path.join(ROOT, "NavySeals", "box_mission_log.csv"),
                    help="box mission CSV (falls back to git commit e5b681a if missing)")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    with open(args.sprint_log) as f:
        sprint = read_log(f.read())
    if args.box_log and os.path.exists(args.box_log):
        with open(args.box_log) as f:
            box_text = f.read()
    else:
        box_text = subprocess.check_output(["git", "show", "e5b681a:NavySeals/mission_log.csv"],
                                           cwd=ROOT).decode()
    box = read_log(box_text)

    sprint_map(sprint, os.path.join(OUT, "sprint_satellite.png"))
    sprint_map(sprint, os.path.join(OUT, "sprint_satellite_wide.png"), aspect=2.3, out_w=2200)  # report
    sprint_map(sprint, os.path.join(OUT, "sprint_clean.png"), aspect=16 / 9, clean=True)  # closing slide
    box_map(box, os.path.join(OUT, "box_satellite.png"))
    telemetry_chart(sprint, os.path.join(OUT, "sprint_telemetry.png"))


if __name__ == "__main__":
    main()
