"""
Generate 4 ArUco tags for FAST-Calib corner markers, max size on A4.

DICT_6X6_250 IDs 1..4 (hardcoded in upstream src/qr_detect.hpp).
A4 canvas @ 300 DPI, tile = 200 mm (5 mm each side, safe for non-borderless
printers). No labels, no corner ticks, no footer — anything extra on the
page invites "scale to fit" scaling errors in the print dialog. Just the
marker, centred.

Print rule: set "Actual size" / "100%" / "Do not scale". After printing,
measure the black tile edge with a ruler — it MUST be 200.0 mm. If it
is not, the print dialog rescaled.
"""
import os
import cv2
import numpy as np

MM_PER_INCH = 25.4
DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
TILE_MM = 200.0
TILE_IDS = [1, 2, 3, 4]
DICT = cv2.aruco.DICT_6X6_250
OUT_DIR = "/home/thor/bess/config/calib/aruco_a4"


def mm_to_px(mm):
    return int(round(mm * DPI / MM_PER_INCH))


A4_W_PX = mm_to_px(A4_W_MM)
A4_H_PX = mm_to_px(A4_H_MM)
# Tile pixel size divisible by 8 (6x6 data + 2 border modules) for crisp edges.
TILE_PX = (mm_to_px(TILE_MM) // 8) * 8

os.makedirs(OUT_DIR, exist_ok=True)
aruco_dict = cv2.aruco.Dictionary_get(DICT)

for tag_id in TILE_IDS:
    canvas = np.full((A4_H_PX, A4_W_PX), 255, dtype=np.uint8)
    tag = cv2.aruco.drawMarker(aruco_dict, tag_id, TILE_PX, borderBits=1)
    x0 = (A4_W_PX - TILE_PX) // 2
    y0 = (A4_H_PX - TILE_PX) // 2
    canvas[y0:y0 + TILE_PX, x0:x0 + TILE_PX] = tag

    out_path = f"{OUT_DIR}/fastcalib_aruco_id{tag_id}_{int(TILE_MM)}mm_A4.png"
    cv2.imwrite(out_path, canvas)
    print(f"wrote {out_path}  {canvas.shape[1]}x{canvas.shape[0]} px  "
          f"tile={TILE_PX}px={TILE_MM}mm")

print(f"\nPrint at 100% / Actual size. Verify with ruler: tile = {TILE_MM} mm.")
