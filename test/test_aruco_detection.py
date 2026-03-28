#!/usr/bin/env python3
"""
ArUco Detection Test - draws colored contours + labels on each marker.

Usage:
    python test_aruco_detection.py <image_path>
    python test_aruco_detection.py                  # uses camera (press 'q' to quit)

Color coding:
    GREEN  = Corner markers (IDs 40-43)
    CYAN   = Robot markers (IDs 1-5)
    YELLOW = Unknown / other IDs
"""

import cv2
import numpy as np
import sys
import os

# -- Config ------------------------------------------------------
ARUCO_DICT = cv2.aruco.DICT_4X4_50       # Must match your printed tags
CORNER_IDS = {40, 41, 42, 43}
ROBOT_IDS  = {1, 2, 3, 4, 5}

CORNER_NAMES = {40: "BL (40)", 41: "BR (41)", 42: "TR (42)", 43: "TL (43)"}
ROBOT_NAMES  = {i: f"R{i}" for i in ROBOT_IDS}

# Colors (BGR)
GREEN  = (0, 255, 0)
CYAN   = (255, 255, 0)
YELLOW = (0, 255, 255)
RED    = (0, 0, 255)
WHITE  = (255, 255, 255)


def classify_marker(marker_id):
    """Return (label, color) for a given marker ID."""
    if marker_id in CORNER_IDS:
        return CORNER_NAMES.get(marker_id, f"Corner {marker_id}"), GREEN
    elif marker_id in ROBOT_IDS:
        return ROBOT_NAMES.get(marker_id, f"Robot {marker_id}"), CYAN
    else:
        return f"ID {marker_id}", YELLOW


def annotate_frame(frame):
    """Detect ArUco markers and draw contours + labels. Returns annotated frame + stats."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    # Tune for overhead, well-lit arena
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.01   # catch small distant markers
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05

    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, rejected = detector.detectMarkers(gray)

    out = frame.copy()
    stats = {"corners": [], "robots": [], "unknown": [], "total": 0}

    if ids is not None:
        stats["total"] = len(ids)
        for i, marker_id in enumerate(ids.flatten()):
            mid = int(marker_id)
            pts = corners[i][0].astype(int)          # 4 corner points
            label, color = classify_marker(mid)

            # -- Thick contour polygon --
            cv2.polylines(out, [pts], isClosed=True, color=color, thickness=3)

            # -- Corner dots --
            for p in pts:
                cv2.circle(out, tuple(p), 5, RED, -1)

            # -- Centre dot --
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            cv2.circle(out, (cx, cy), 6, color, -1)

            # -- Heading arrow (top-left -> top-right edge) --
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
            angle_deg = np.degrees(np.arctan2(-dy, dx))
            arrow_len = int(np.linalg.norm(pts[1] - pts[0]) * 0.7)
            ax = int(cx + arrow_len * np.cos(np.radians(-angle_deg)))
            ay = int(cy + arrow_len * np.sin(np.radians(-angle_deg)))
            cv2.arrowedLine(out, (cx, cy), (ax, ay), WHITE, 2, tipLength=0.3)

            # -- Label text --
            # background rectangle for readability
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            tx, ty = cx - tw // 2, cy - 25
            cv2.rectangle(out, (tx - 2, ty - th - 4), (tx + tw + 2, ty + 4),
                          (0, 0, 0), -1)
            cv2.putText(out, label, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # -- Pixel coords under marker --
            coord_str = f"({cx}, {cy})"
            cv2.putText(out, coord_str, (cx - 40, cy + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1)

            # -- Stats --
            if mid in CORNER_IDS:
                stats["corners"].append(mid)
            elif mid in ROBOT_IDS:
                stats["robots"].append(mid)
            else:
                stats["unknown"].append(mid)

    # -- Rejected regions (faint) --
    for rej in (rejected or []):
        pts = rej[0].astype(int)
        cv2.polylines(out, [pts], True, (80, 80, 80), 1)

    # -- HUD --
    y0 = 30
    lines = [
        f"Detected: {stats['total']} markers",
        f"Corners : {sorted(stats['corners'])}  ({len(stats['corners'])}/4)",
        f"Robots  : {sorted(stats['robots'])}  ({len(stats['robots'])}/5)",
    ]
    if stats["unknown"]:
        lines.append(f"Unknown : {sorted(stats['unknown'])}")

    for j, txt in enumerate(lines):
        cv2.putText(out, txt, (15, y0 + j * 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2)

    # Pass / Fail
    all_corners = len(stats["corners"]) == 4
    all_robots  = len(stats["robots"]) == 5
    if all_corners and all_robots:
        status, status_col = "ALL 9 MARKERS DETECTED", GREEN
    else:
        missing = []
        for cid in sorted(CORNER_IDS):
            if cid not in stats["corners"]:
                missing.append(CORNER_NAMES[cid])
        for rid in sorted(ROBOT_IDS):
            if rid not in stats["robots"]:
                missing.append(f"R{rid}")
        status = f"MISSING: {', '.join(missing)}"
        status_col = RED

    cv2.putText(out, status, (15, y0 + len(lines) * 30 + 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_col, 2)

    return out, stats


# -- Main --------------------------------------------------------
def main():
    if len(sys.argv) > 1 and os.path.isfile(sys.argv[1]):
        # -- Static image mode --
        img_path = sys.argv[1]
        print(f"Loading image: {img_path}")
        img = cv2.imread(img_path)
        if img is None:
            print(f"ERROR: cannot read {img_path}")
            sys.exit(1)

        annotated, stats = annotate_frame(img)

        # Print summary
        print(f"\n{'='*50}")
        print(f"  ArUco Detection Results")
        print(f"{'='*50}")
        print(f"  Total detected : {stats['total']}")
        print(f"  Corner markers : {sorted(stats['corners'])}  ({len(stats['corners'])}/4)")
        print(f"  Robot markers  : {sorted(stats['robots'])}  ({len(stats['robots'])}/5)")
        if stats["unknown"]:
            print(f"  Unknown IDs    : {sorted(stats['unknown'])}")
        print(f"{'='*50}\n")

        # Save annotated image
        out_path = os.path.splitext(img_path)[0] + "_aruco_annotated.png"
        cv2.imwrite(out_path, annotated)
        print(f"Saved: {out_path}")

        # Show
        cv2.imshow("ArUco Detection Test", annotated)
        print("Press any key to close...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    else:
        # -- Camera mode --
        cam_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
        print(f"Opening camera {cam_id}... (press 'q' to quit)")
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not cap.isOpened():
            print(f"ERROR: cannot open camera {cam_id}")
            sys.exit(1)

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            annotated, stats = annotate_frame(frame)
            cv2.imshow("ArUco Detection Test", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # Print live stats every ~1s
            if cv2.waitKey(1) & 0xFF == ord('s'):
                print(f"Corners: {sorted(stats['corners'])}  Robots: {sorted(stats['robots'])}")

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
