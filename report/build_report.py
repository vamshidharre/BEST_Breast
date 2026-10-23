#!/usr/bin/env python3
"""
Builds report/NavySeals_BEST2026_Report.pdf from report/navyseals_report.html
with headless Chrome (or Edge), then checks it is exactly 4 A4 pages with no overflow.

Run from anywhere:  python report/build_report.py
Re-run after adding boat1.jpg to the repo root so the photo appears in section 2.
"""

import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(HERE, "navyseals_report.html")
PDF = os.path.join(HERE, "NavySeals_BEST2026_Report.pdf")

BROWSERS = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    "/usr/bin/google-chrome", "/usr/bin/chromium", "/usr/bin/chromium-browser",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
]


def find_browser():
    for b in BROWSERS:
        if os.path.exists(b):
            return b
    sys.exit("No Chrome/Edge found; edit BROWSERS in build_report.py")


def main():
    browser = find_browser()
    url = "file:///" + HTML.replace("\\", "/")
    profile = tempfile.mkdtemp(prefix="report-chrome-")
    common = [browser, "--headless=new", "--disable-gpu", "--user-data-dir=" + profile,
              "--virtual-time-budget=8000", "--run-all-compositor-stages-before-draw"]

    # 1. layout check
    dom = subprocess.run(common + ["--dump-dom", url], capture_output=True, text=True,
                         encoding="utf-8", errors="replace").stdout
    m = re.search(r'data-overflow="([^"]*)"', dom)
    overflow = m.group(1) if m else "unknown"
    print("Overflow check:", overflow)

    # 2. PDF
    subprocess.run(common + ["--no-pdf-header-footer", "--print-to-pdf=" + PDF, url],
                   capture_output=True)
    try:
        from pypdf import PdfReader
        pages = len(PdfReader(PDF).pages)
        print("Wrote {} ({} pages)".format(os.path.relpath(PDF), pages))
        if pages != 4:
            print("WARNING: expected 4 pages")
    except ImportError:
        print("Wrote", PDF, "(install pypdf to check the page count)")
    if overflow not in ("none",):
        print("WARNING: content overflow on page(s):", overflow)


if __name__ == "__main__":
    main()
