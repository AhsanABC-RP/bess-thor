"""
Foil cutting templates for the 4 FAST-Calib ArUco tags (thermal workflow).

For each ID in 1..4, produces an A4 PNG identical in geometry to the paper
ArUco (200 mm tile, centred, no margins/labels on the tile itself) plus:
  - thin module grid overlaid on the tile (25 mm × 25 mm squares — 8×8
    modules total = 6 data + 2-module black border)
  - page-edge label "FOIL TEMPLATE id=N" and "black = foil, white = bare"

Workflow:
  1. Print the PAPER sheet (fastcalib_aruco_id{N}_200mm_A4.png) at 100%.
  2. Print THIS foil-template sheet at 100%.
  3. Apply aluminium foil tape (emissivity ~0.03) edge-to-edge across the
     200 mm × 200 mm tile area of the foil template, smooth out bubbles.
  4. With a sharp knife + steel ruler, cut along the 25 mm grid lines and
     peel foil off every WHITE module, leaving foil only on the BLACK ones.
  5. Transfer the foil stencil onto the paper ArUco (matched pattern) OR
     glue the foil-stencilled sheet directly to the target. LWIR sees
     foil (cold reflection) vs paper (ambient emission) → ArUco contrast.

Each tile module is 25 mm — keep the knife straight, the detector needs
crisp edges.
"""
import os
import cv2
import numpy as np

MM_PER_INCH = 25.4
DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
TILE_MM = 200.0
MODULES_TOTAL = 8          # 6 data + 2 border modules (upstream uses borderBits=1 inside drawMarker)
TILE_IDS = [1, 2, 3, 4]
DICT = cv2.aruco.DICT_6X6_250
OUT_DIR = "/home/thor/bess/config/calib/aruco_a4_foil_template"


def mm_to_px(mm):
    return int(round(mm * DPI / MM_PER_INCH))


A4_W_PX = mm_to_px(A4_W_MM)
A4_H_PX = mm_to_px(A4_H_MM)
TILE_PX = (mm_to_px(TILE_MM) // MODULES_TOTAL) * MODULES_TOTAL
MODULE_PX = TILE_PX // MODULES_TOTAL
MODULE_MM = TILE_MM / MODULES_TOTAL

os.makedirs(OUT_DIR, exist_ok=True)
aruco_dict = cv2.aruco.Dictionary_get(DICT)

for tag_id in TILE_IDS:
    canvas = np.full((A4_H_PX, A4_W_PX), 255, dtype=np.uint8)
    tag = cv2.aruco.drawMarker(aruco_dict, tag_id, TILE_PX, borderBits=1)
    x0 = (A4_W_PX - TILE_PX) // 2
    y0 = (A4_H_PX - TILE_PX) // 2
    canvas[y0:y0 + TILE_PX, x0:x0 + TILE_PX] = tag

    # Light red-ish grid (stays grey in B&W print) at every module boundary.
    # Using mid-grey (160) so the cutter can see the line but detection on the
    # paper version is unaffected if someone re-prints this by accident.
    grid_col = 160
    for i in range(MODULES_TOTAL + 1):
        xi = x0 + i * MODULE_PX
        yi = y0 + i * MODULE_PX
        cv2.line(canvas, (xi, y0), (xi, y0 + TILE_PX), grid_col, 1)
        cv2.line(canvas, (x0, yi), (x0 + TILE_PX, yi), grid_col, 1)

    header = f"FOIL TEMPLATE  id={tag_id}  tile={int(TILE_MM)}mm  module={MODULE_MM:.1f}mm"
    legend = "black = FOIL (cut & keep)   white = BARE (cut & peel)"
    cv2.putText(canvas, header, (x0, y0 - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2, cv2.LINE_AA)
    cv2.putText(canvas, legend, (x0, y0 + TILE_PX + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 1, cv2.LINE_AA)

    out_path = f"{OUT_DIR}/fastcalib_foil_id{tag_id}_{int(TILE_MM)}mm_A4.png"
    cv2.imwrite(out_path, canvas)
    print(f"wrote {out_path}  tile={TILE_PX}px={TILE_MM}mm  "
          f"module={MODULE_PX}px={MODULE_MM:.2f}mm")

print(f"\nPrint at 100% / Actual size. Verify: tile edge = {TILE_MM:.0f} mm "
      f"and one grid cell = {MODULE_MM:.1f} mm.")
