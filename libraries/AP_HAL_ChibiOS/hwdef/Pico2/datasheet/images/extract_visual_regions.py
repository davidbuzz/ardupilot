#!/usr/bin/env python3
"""
RP2350 Datasheet — Visual Region Extractor
==========================================
For each page:
  1. Load the rasterized PNG
  2. Ask pdfplumber for all text bounding boxes on that page
  3. Paint a "text mask" white over the image copy
  4. Detect connected non-white, non-background regions that survive the mask
  5. If a region is large enough → it's a diagram / figure / table-as-graphic
  6. Crop it out and save as  visuals/page_XXXX_fig_NN.png
  7. Write an index JSON mapping each crop to its page and position

Strategy for "is this region non-text?":
  - Build a grayscale version of the masked image
  - Use a sliding-window or contour approach to find rectangular bounding
    boxes of non-white content that are NOT already explained by text chars
  - Minimum region size: 80px wide AND 60px tall (filters out stray lines,
    bullet points, horizontal rules)
  - Merge overlapping / nearby candidate boxes (within 20px gap) so a
    multi-part diagram becomes one crop
"""

import os
import sys
import json
import argparse
import traceback
from pathlib import Path

import pdfplumber
from PIL import Image, ImageDraw, ImageFilter
import numpy as np

# ── Config ─────────────────────────────────────────────────────────────────
PDF_PATH      = "/mnt/user-data/uploads/RP-008373-DS-2-rp2350-datasheet.pdf"
PAGES_DIR     = "/home/claude/pages"
OUTPUT_DIR    = "/home/claude/visuals"
INDEX_PATH    = "/home/claude/visuals_index.json"

DPI           = 150          # must match how pages/ was rasterised
PDF_DPI       = 72           # PDF coordinate space
SCALE         = DPI / PDF_DPI   # pts → pixels  (≈2.083)

# Text-mask padding (px) — expands each word box slightly so anti-aliased
# glyph edges are also covered
TEXT_PAD      = 4

# Minimum size (px) for a visual region to be saved
MIN_W         = 80
MIN_H         = 60

# Gap (px) — nearby boxes closer than this are merged into one region
MERGE_GAP     = 20

# Background brightness threshold — pixels brighter than this are "white"
BG_THRESH     = 245

# Margin strips to ignore (header/footer rules, page number lines)
# expressed as fraction of page height
IGNORE_TOP    = 0.06
IGNORE_BOTTOM = 0.06

# ── Helpers ────────────────────────────────────────────────────────────────

def page_img_path(page_num_1indexed: int) -> str:
    return os.path.join(PAGES_DIR, f"page-{page_num_1indexed:04d}.png")


def get_text_boxes(pdf_page, img_w: int, img_h: int):
    """Return list of (x0,y0,x1,y1) pixel rects for every word on the page."""
    pw, ph = pdf_page.width, pdf_page.height
    sx = img_w / pw
    sy = img_h / ph
    boxes = []
    for w in pdf_page.extract_words(extra_attrs=["size"]):
        x0 = max(0, int(w["x0"] * sx) - TEXT_PAD)
        y0 = max(0, int(w["top"] * sy) - TEXT_PAD)
        x1 = min(img_w, int(w["x1"] * sx) + TEXT_PAD)
        y1 = min(img_h, int(w["bottom"] * sy) + TEXT_PAD)
        boxes.append((x0, y0, x1, y1))
    return boxes


def build_text_mask(img_size, text_boxes):
    """Boolean mask: True where text lives."""
    mask = np.zeros((img_size[1], img_size[0]), dtype=bool)
    for (x0, y0, x1, y1) in text_boxes:
        mask[y0:y1, x0:x1] = True
    return mask


def find_content_rows(grey_arr, text_mask, img_h):
    """
    For each row, check if there are dark pixels outside the text mask.
    Returns a boolean array of shape (img_h,).
    """
    dark = grey_arr < BG_THRESH          # True where pixel is not white
    non_text_dark = dark & ~text_mask    # dark AND not masked by text
    return non_text_dark.any(axis=1)     # True for rows with visual content


def rows_to_bands(row_signal, merge_gap):
    """
    Convert boolean row array into list of (y_start, y_end) contiguous bands,
    merging gaps smaller than merge_gap rows.
    """
    bands = []
    in_band = False
    start = 0
    gap_count = 0

    for i, val in enumerate(row_signal):
        if val:
            if not in_band:
                if bands and (i - bands[-1][1]) <= merge_gap:
                    # extend last band
                    start = bands[-1][0]
                    bands.pop()
                else:
                    start = i
                in_band = True
            gap_count = 0
        else:
            if in_band:
                gap_count += 1
                if gap_count > merge_gap:
                    bands.append((start, i - gap_count))
                    in_band = False
                    gap_count = 0

    if in_band:
        bands.append((start, len(row_signal) - 1))

    return bands


def band_to_tight_box(grey_arr, text_mask, y0, y1, img_w):
    """
    Within a horizontal band, find the tightest x extent of non-text dark pixels.
    Returns (x0, y0, x1, y1) or None if nothing significant.
    """
    strip = grey_arr[y0:y1, :]
    tmask = text_mask[y0:y1, :]
    dark_strip = (strip < BG_THRESH) & ~tmask

    cols_with_content = dark_strip.any(axis=0)
    if not cols_with_content.any():
        return None

    xs = np.where(cols_with_content)[0]
    x0 = max(0, int(xs[0]) - MERGE_GAP)
    x1 = min(img_w, int(xs[-1]) + MERGE_GAP)

    return (x0, y0, x1, y1)


def merge_boxes(boxes, gap):
    """Merge overlapping or nearby (x0,y0,x1,y1) boxes."""
    if not boxes:
        return []
    boxes = sorted(boxes, key=lambda b: (b[1], b[0]))
    merged = [list(boxes[0])]
    for bx, by, bx2, by2 in boxes[1:]:
        m = merged[-1]
        if by <= m[3] + gap and bx <= m[2] + gap:
            m[0] = min(m[0], bx)
            m[1] = min(m[1], by)
            m[2] = max(m[2], bx2)
            m[3] = max(m[3], by2)
        else:
            merged.append([bx, by, bx2, by2])
    return [tuple(b) for b in merged]


def extract_visuals_from_page(pdf_page, page_num, img_path, out_dir):
    """
    Main per-page extraction. Returns list of dicts describing saved crops.
    """
    img = Image.open(img_path).convert("RGB")
    img_w, img_h = img.size

    # Ignore header / footer strips
    ignore_top_px    = int(img_h * IGNORE_TOP)
    ignore_bottom_px = int(img_h * (1 - IGNORE_BOTTOM))

    text_boxes = get_text_boxes(pdf_page, img_w, img_h)
    text_mask  = build_text_mask((img_w, img_h), text_boxes)

    grey_arr = np.array(img.convert("L"))

    # Zero out ignore zones in the row signal
    row_signal = find_content_rows(grey_arr, text_mask, img_h)
    row_signal[:ignore_top_px]    = False
    row_signal[ignore_bottom_px:] = False

    bands = rows_to_bands(row_signal, MERGE_GAP)

    raw_boxes = []
    for (y0, y1) in bands:
        box = band_to_tight_box(grey_arr, text_mask, y0, y1, img_w)
        if box:
            bw = box[2] - box[0]
            bh = box[3] - box[1]
            if bw >= MIN_W and bh >= MIN_H:
                raw_boxes.append(box)

    final_boxes = merge_boxes(raw_boxes, MERGE_GAP)

    crops = []
    for fig_idx, (x0, y0, x1, y1) in enumerate(final_boxes):
        crop = img.crop((x0, y0, x1, y1))
        fname = f"page_{page_num:04d}_fig_{fig_idx+1:02d}.png"
        fpath = os.path.join(out_dir, fname)
        crop.save(fpath, optimize=True)
        crops.append({
            "page": page_num,
            "figure": fig_idx + 1,
            "file": fname,
            "bbox_px": [x0, y0, x1, y1],
            "size_px": [x1 - x0, y1 - y0],
        })

    return crops


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Extract visual (non-text) regions from RP2350 datasheet pages.")
    parser.add_argument("--pages", nargs="*", type=int, help="Specific 1-indexed page numbers to process (default: all)")
    parser.add_argument("--start", type=int, default=1, help="Start page (inclusive)")
    parser.add_argument("--end",   type=int, default=1380, help="End page (inclusive)")
    args = parser.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    index = []

    with pdfplumber.open(PDF_PATH) as pdf:
        total_pages = len(pdf.pages)

        if args.pages:
            page_nums = sorted(set(args.pages))
        else:
            page_nums = list(range(args.start, min(args.end, total_pages) + 1))

        print(f"Processing {len(page_nums)} pages → {OUTPUT_DIR}")
        print(f"{'Page':>6}  {'Figs':>4}  Status")
        print("-" * 30)

        for page_num in page_nums:
            img_path = page_img_path(page_num)
            if not os.path.exists(img_path):
                print(f"{page_num:6d}  {'?':>4}  SKIP (no image)")
                continue

            try:
                pdf_page = pdf.pages[page_num - 1]
                crops = extract_visuals_from_page(pdf_page, page_num, img_path, OUTPUT_DIR)
                index.extend(crops)
                print(f"{page_num:6d}  {len(crops):>4}  ok")
            except Exception as e:
                print(f"{page_num:6d}  {'!':>4}  ERROR: {e}")
                traceback.print_exc()

    with open(INDEX_PATH, "w") as f:
        json.dump(index, f, indent=2)

    print(f"\nDone. {len(index)} visual regions saved.")
    print(f"Index → {INDEX_PATH}")


if __name__ == "__main__":
    main()
