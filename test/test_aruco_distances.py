#!/usr/bin/env python3
"""
ArUco Distance & Position Measurement.

Deployment-friendly overlay: just robot ID labels + corner labels.
All position/distance/heading data is saved to a JSON file for post-processing.

Usage:
    python test_aruco_distances.py <image_path>
    python test_aruco_distances.py              # live camera ('s' to snapshot, 'q' to quit)
"""

import cv2
import numpy as np
import sys
import os
import math
import json
import time

# ── Config (must match aruco_position_server.py) ────────────────
ARUCO_DICT = cv2.aruco.DICT_4X4_50
CORNER_IDS = {40: "BL", 41: "BR", 42: "TR", 43: "TL"}
ROBOT_IDS  = {1, 2, 3, 4, 5}

# Arena dimensions in mm
ARENA_W_MM = 2000
ARENA_H_MM = 1500

# Corner marker → real-world coordinate mapping (mm)
# Arena origin (0,0) = inner bottom-left of the playable black surface.
# Tags sit on the outer blue-tape border, offset only in X (not Y).
CORNER_ARENA = {
    40: (-115,     0),   # BL — 11.5 cm left  of origin,  level with y=0
    41: (2110,     0),   # BR — 211  cm right of origin,  level with y=0
    42: (2110,  1500),   # TR — 11   cm right of arena,   level with y=1500
    43: (-113,  1500),   # TL — 11.3 cm left  of origin,  level with y=1500
}

# ── Colors (BGR) ────────────────────────────────────────────────
GREEN   = (0, 255, 0)
CYAN    = (255, 255, 0)
WHITE   = (255, 255, 255)
YELLOW  = (0, 255, 255)
BLACK   = (0, 0, 0)

# Robot ID → display colour
ROBOT_COLORS = {
    1: (0, 200, 255),   # orange-ish
    2: (255, 100, 100),  # blue-ish
    3: (100, 255, 100),  # green
    4: (255, 100, 255),  # magenta
    5: (100, 255, 255),  # yellow-ish
}

# Serif-style font (closest to IEEE/Times in OpenCV's built-in set)
FONT = cv2.FONT_HERSHEY_COMPLEX


def detect_markers(frame):
    """Detect all ArUco markers, return dict of id -> info."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.01
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(gray)

    markers = {}
    if ids is not None:
        for i, mid in enumerate(ids.flatten()):
            pts = corners[i][0]
            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
            theta = math.atan2(-dy, dx)
            markers[int(mid)] = {
                'px': (cx, cy),
                'corners_px': pts,
                'theta_rad': theta,
            }
    return markers


def compute_homography(markers):
    """Build pixel→mm homography from 4 corner markers."""
    found = [cid for cid in CORNER_IDS if cid in markers]
    if len(found) < 4:
        missing = [c for c in CORNER_IDS if c not in markers]
        print(f"  WARNING: only {len(found)}/4 corner markers — missing {missing}")
        return None
    src = np.array([markers[cid]['px'] for cid in [40, 41, 42, 43]],
                   dtype=np.float32)
    dst = np.array([CORNER_ARENA[cid] for cid in [40, 41, 42, 43]],
                   dtype=np.float32)
    H, _ = cv2.findHomography(src, dst)
    return H


def px_to_mm(px, py, H):
    pt = np.array([[[px, py]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H)
    return float(out[0][0][0]), float(out[0][0][1])


def dist_mm(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ── Deployment overlay (clean) ──────────────────────────────────
def annotate_clean(frame, markers, H):
    """Minimal deployment overlay: robot IDs + corner labels only."""
    out = frame.copy()
    has_H = H is not None

    # Corner markers: small green dot + label
    for cid, label in CORNER_IDS.items():
        if cid not in markers:
            continue
        px = tuple(int(v) for v in markers[cid]['px'])
        cv2.circle(out, px, 6, GREEN, -1)
        cv2.putText(out, label, (px[0] + 10, px[1] - 8),
                    FONT, 0.55, GREEN, 2)

    # Robot markers: coloured circle + ID label
    for rid in sorted(ROBOT_IDS):
        if rid not in markers:
            continue
        m = markers[rid]
        px = tuple(int(v) for v in m['px'])
        col = ROBOT_COLORS.get(rid, CYAN)

        # Filled circle with black border
        cv2.circle(out, px, 14, BLACK, -1)
        cv2.circle(out, px, 14, col, 2)

        # ID label
        label = f"R{rid}"
        if has_H:
            ax, ay = px_to_mm(m['px'][0], m['px'][1], H)
            label += f" ({ax:.0f},{ay:.0f})"
        (tw, th), _ = cv2.getTextSize(label, FONT, 0.55, 2)
        lx, ly = px[0] - tw // 2, px[1] - 22
        cv2.rectangle(out, (lx - 3, ly - th - 3), (lx + tw + 3, ly + 5), BLACK, -1)
        cv2.putText(out, label, (lx, ly),
                    FONT, 0.55, col, 2)

        # No heading arrows — keep overlay minimal for deployment

    # Minimal HUD — centred at top so corner labels stay visible
    n_corners = sum(1 for c in CORNER_IDS if c in markers)
    n_robots  = sum(1 for r in ROBOT_IDS if r in markers)
    mode = "HOMOGRAPHY" if has_H else "NO HOMOGRAPHY"
    hud = f"Corners: {n_corners}/4 | Robots: {n_robots}/5 | {mode}"
    (tw, th), _ = cv2.getTextSize(hud, FONT, 0.6, 2)
    hud_x = (out.shape[1] - tw) // 2
    cv2.rectangle(out, (hud_x - 6, 6), (hud_x + tw + 6, 34), BLACK, -1)
    cv2.putText(out, hud, (hud_x, 28),
                FONT, 0.6, YELLOW, 2)

    return out


# ── JSON export ─────────────────────────────────────────────────
def build_json(markers, H):
    """Build a dict with all positions, distances, headings for post-processing."""
    has_H = H is not None
    unit = "mm" if has_H else "px"
    data = {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'unit': unit,
        'arena_mm': {'width': ARENA_W_MM, 'height': ARENA_H_MM},
        'homography_ok': has_H,
        'corners': {},
        'robots': {},
        'pairwise_distances': {},
    }

    # Arena coords for all markers
    arena = {}
    for mid, m in markers.items():
        if has_H:
            arena[mid] = px_to_mm(m['px'][0], m['px'][1], H)
        else:
            arena[mid] = m['px']

    # Corner markers
    for cid, label in CORNER_IDS.items():
        if cid not in markers:
            continue
        ax, ay = arena[cid]
        data['corners'][label] = {
            'id': cid,
            'x': round(ax, 1), 'y': round(ay, 1),
            'px': list(markers[cid]['px']),
        }

    # Robot markers
    for rid in sorted(ROBOT_IDS):
        if rid not in markers:
            continue
        m = markers[rid]
        ax, ay = arena[rid]
        heading = math.degrees(m['theta_rad'])
        rdata = {
            'x': round(ax, 1), 'y': round(ay, 1),
            'heading_deg': round(heading, 1),
            'px': [round(v, 1) for v in m['px']],
        }
        if has_H:
            rdata['wall_distances'] = {
                'left':   round(ax, 1),
                'right':  round(ARENA_W_MM - ax, 1),
                'bottom': round(ay, 1),
                'top':    round(ARENA_H_MM - ay, 1),
            }
        data['robots'][f'R{rid}'] = rdata

    # Pairwise robot distances
    robot_ids = sorted([r for r in ROBOT_IDS if r in markers])
    for i in range(len(robot_ids)):
        for j in range(i + 1, len(robot_ids)):
            ra, rb = robot_ids[i], robot_ids[j]
            d = dist_mm(arena[ra], arena[rb])
            data['pairwise_distances'][f'R{ra}-R{rb}'] = round(d, 1)

    # Summary
    if len(robot_ids) >= 2:
        dists = []
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                d = dist_mm(arena[robot_ids[i]], arena[robot_ids[j]])
                dists.append((robot_ids[i], robot_ids[j], d))
        dists.sort(key=lambda x: x[2])
        data['closest_pair'] = {
            'pair': f'R{dists[0][0]}-R{dists[0][1]}',
            'distance': round(dists[0][2], 1),
        }
        data['farthest_pair'] = {
            'pair': f'R{dists[-1][0]}-R{dists[-1][1]}',
            'distance': round(dists[-1][2], 1),
        }

    return data


def print_summary(data):
    """Quick console summary."""
    unit = data['unit']
    print(f"\n{'='*55}")
    print(f"  Robot Positions ({unit})   [{data['timestamp']}]")
    print(f"{'='*55}")
    for rname, rd in data['robots'].items():
        print(f"  {rname}:  x={rd['x']:7.1f}  y={rd['y']:7.1f}  hdg={rd['heading_deg']:6.1f} deg")

    if data['pairwise_distances']:
        print(f"\n  Pairwise Distances ({unit}):")
        for pair, d in data['pairwise_distances'].items():
            print(f"    {pair}: {d:.0f}")

    if 'closest_pair' in data:
        c = data['closest_pair']
        f = data['farthest_pair']
        print(f"\n  Closest:  {c['pair']} = {c['distance']:.0f} {unit}")
        print(f"  Farthest: {f['pair']} = {f['distance']:.0f} {unit}")
    print(f"{'='*55}\n")


# ── Main ────────────────────────────────────────────────────────
def main():
    if len(sys.argv) > 1 and os.path.isfile(sys.argv[1]):
        img_path = sys.argv[1]
        print(f"Loading: {img_path}")
        img = cv2.imread(img_path)
        if img is None:
            print(f"ERROR: cannot read {img_path}")
            sys.exit(1)

        markers = detect_markers(img)
        H = compute_homography(markers)
        data = build_json(markers, H)
        print_summary(data)

        # Save JSON
        json_path = os.path.splitext(img_path)[0] + "_aruco.json"
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved JSON: {json_path}")

        # Save annotated image
        annotated = annotate_clean(img, markers, H)
        out_path = os.path.splitext(img_path)[0] + "_annotated.png"
        cv2.imwrite(out_path, annotated)
        print(f"Saved image: {out_path}")

        cv2.imshow("ArUco", annotated)
        print("Press any key to close...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    else:
        cam_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
        print(f"Camera {cam_id}... 's'=snapshot  'q'=quit")
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if not cap.isOpened():
            print(f"ERROR: cannot open camera {cam_id}")
            sys.exit(1)

        snap_count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            markers = detect_markers(frame)
            H = compute_homography(markers)
            annotated = annotate_clean(frame, markers, H)
            cv2.imshow("ArUco", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                snap_count += 1
                data = build_json(markers, H)
                print_summary(data)
                json_path = f"snapshot_{snap_count:03d}_aruco.json"
                with open(json_path, 'w') as f:
                    json.dump(data, f, indent=2)
                print(f"  -> {json_path}")

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
