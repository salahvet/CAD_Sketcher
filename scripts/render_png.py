"""Rend les SVG en PNG transparents (Chromium via Playwright).

    python render_png.py <dossier_svg> <dossier_png>

Contraintes (nom en MAJUSCULES) : 400 px, comme les PNG d'origine.
Autres icones : 128 px.
"""
import sys
from pathlib import Path
from playwright.sync_api import sync_playwright

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
dst.mkdir(parents=True, exist_ok=True)
with sync_playwright() as p:
    b = p.chromium.launch()
    for f in sorted(src.glob("*.svg")):
        size = 400 if f.stem.isupper() else 128
        page = b.new_page(viewport={"width": size, "height": size})
        page.set_content(
            '<html><body style="margin:0;background:transparent">'
            f'<div style="width:{size}px;height:{size}px">'
            + f.read_text(encoding="utf-8").replace(
                'width="256" height="256"', f'width="{size}" height="{size}"')
            + "</div></body></html>")
        page.screenshot(path=str(dst / (f.stem + ".png")), omit_background=True)
        page.close()
    b.close()
print("ok", len(list(dst.glob("*.png"))))
