#!/usr/bin/env python3
"""
REIP ArUco Position Server
Detects robot positions and sends each robot ONLY its own position.
Acts as a "GPS satellite" - provides localization, nothing else.

Usage: python3 aruco_position_server.py [camera_id]
"""

import cv2
import numpy as np
import time
import json
import socket
import threading
import os
import math
from datetime import datetime
from typing import Dict, Tuple, Optional

# ============== Configuration ==============
CAMERA_ID = 0
CAMERA_WIDTH = 1280   # 720p — faster ArUco detection, higher FPS for position updates
CAMERA_HEIGHT = 720

# ArUco
ARUCO_DICT = cv2.aruco.DICT_4X4_50  # IDs 0-49 available

# Corner markers for auto-calibration (place at arena corners)
# Using IDs 40-43 (robots use 1-5, corners use 40-43)
# 40 = bottom-left, 41 = bottom-right, 42 = top-right, 43 = top-left
CORNER_MARKER_IDS = [40, 41, 42, 43]

# Arena - must match reip_node.py
ARENA_WIDTH_MM = 2000   # mm
ARENA_HEIGHT_MM = 1500  # mm (16:12 aspect, ~matches 16:9 camera)

# Network - MUST MATCH robot/reip_node.py
# Hardware mode: all robots listen on UDP_PORT (5100).
# Messages include robot_id; each robot filters for its own.
UDP_PORT = 5100  # Position updates to robots
UDP_PEER_PORT = 5200  # Robot peer-to-peer broadcasts (we eavesdrop)
BROADCAST_IP = "192.168.20.255"
POSITION_RATE = 30  # Hz

# Robot IPs (None = broadcast to all, or set specific IPs for unicast)
ROBOT_IPS = {
    1: None,  # None = broadcast, or set "192.168.1.101"
    2: None,
    3: None,
    4: None,
    5: None,
    6: None,
}

# ============== Tracker ==============
class PositionServer:
    def __init__(self, camera_id: int = CAMERA_ID):
        print("=== REIP Position Server ===")
        print("(GPS satellite mode - each robot gets only its own position)\n")
        
        print(f"Opening camera {camera_id}...")
        # Try each backend; DirectShow is fastest on Windows when it works
        for backend_name, backend in [("DirectShow", cv2.CAP_DSHOW),
                                       ("MSMF", cv2.CAP_MSMF),
                                       ("Default", cv2.CAP_ANY)]:
            print(f"  Trying {backend_name}...", end=" ", flush=True)
            self.cap = cv2.VideoCapture(camera_id, backend)
            if self.cap.isOpened():
                print("OK")
                break
            else:
                print("failed")
                self.cap.release()
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_id}")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -7)
        
        ret, test_frame = self.cap.read()
        if ret:
            actual_h, actual_w = test_frame.shape[:2]
            print(f"Camera: {actual_w}x{actual_h}")
        else:
            actual_w = CAMERA_WIDTH
            actual_h = CAMERA_HEIGHT
            print("Warning: camera opened but no frames yet")
        
        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Calibration - two modes:
        # 1. Homography from 4 corner markers (preferred, auto-detects each frame)
        # 2. Simple linear scaling (fallback)
        self.homography = None      # 3x3 pixel→arena transform
        self.inv_homography = None  # 3x3 arena→pixel (for overlay drawing)
        self.use_homography = True  # Try to use corner markers
        
        # Fallback linear calibration
        self.pixels_per_mm_x = actual_w / ARENA_WIDTH_MM
        self.pixels_per_mm_y = actual_h / ARENA_HEIGHT_MM
        self.origin_x = 0
        self.origin_y = actual_h
        
        # Network — bind to WiFi interface so broadcasts reach robots
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.bind(("192.168.20.214", 0))
        
        # State
        self.running = False
        self.show_video = True
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()

        # Robot state listener (eavesdrop on peer broadcasts)
        self.robot_states: Dict[int, dict] = {}
        self._state_lock = threading.Lock()
        self._start_state_listener()

        # Video recording
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.recording = False
        self.record_path = ""

        # JSON state log (parallel to video)
        self.state_log_file = None
        self.trial_dir = ""

        self.show_exploration = True
        self._visited_cells: set = set()

        print("Ready!\n")
        print("  'r' = start/stop recording   'q' = quit")
        print("  'c' = calibrate              'v' = toggle video")
        print("  'e' = toggle exploration overlay\n")
    
    # ==================== State listener ====================
    def _start_state_listener(self):
        """Listen on UDP_PEER_PORT for robot peer_state broadcasts."""
        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._state_sock.bind(("0.0.0.0", UDP_PEER_PORT))
        self._state_sock.settimeout(0.05)
        t = threading.Thread(target=self._state_listener_loop, daemon=True)
        t.start()

    def _state_listener_loop(self):
        while True:
            try:
                data, _ = self._state_sock.recvfrom(65536)
                msg = json.loads(data.decode())
                if msg.get('type') == 'peer_state':
                    rid = msg.get('robot_id')
                    if rid:
                        with self._state_lock:
                            self.robot_states[rid] = msg
            except socket.timeout:
                pass
            except Exception:
                pass

    # ==================== Recording (background thread) ====================
    def start_recording(self, frame_shape):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.trial_dir = os.path.join("trials", ts)
        os.makedirs(self.trial_dir, exist_ok=True)

        self.record_path = os.path.join(self.trial_dir, "overhead.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        h, w = frame_shape[:2]
        self.video_writer = cv2.VideoWriter(self.record_path, fourcc, 30.0, (w, h))
        self.recording = True
        self._rec_queue: list = []
        self._rec_lock = threading.Lock()
        self._rec_stop = threading.Event()
        self._rec_thread = threading.Thread(target=self._rec_writer_loop, daemon=True)
        self._rec_thread.start()

        log_path = os.path.join(self.trial_dir, "robot_states.jsonl")
        self.state_log_file = open(log_path, 'w')

        print(f"[REC] Recording to {self.trial_dir}")

    def _rec_writer_loop(self):
        """Background thread that writes frames + state lines without blocking the main loop."""
        while not self._rec_stop.is_set():
            with self._rec_lock:
                batch = self._rec_queue[:]
                self._rec_queue.clear()
            for frame_copy, state_line in batch:
                if self.video_writer:
                    self.video_writer.write(frame_copy)
                if self.state_log_file and state_line:
                    self.state_log_file.write(state_line)
            if not batch:
                time.sleep(0.005)

    def _enqueue_frame(self, frame, poses):
        """Queue a frame + state for background writing."""
        with self._state_lock:
            states_snapshot = dict(self.robot_states)
        record = {
            't': time.time(),
            'poses': {str(k): {'x': float(v['x']), 'y': float(v['y']), 'theta': float(v['theta'])}
                      for k, v in poses.items()},
            'robot_states': {}
        }
        for rid, st in states_snapshot.items():
            record['robot_states'][str(rid)] = {
                'x': st.get('x'), 'y': st.get('y'), 'theta': st.get('theta'),
                'state': st.get('state'),
                'leader_id': st.get('leader_id'),
                'trust_in_leader': st.get('trust_in_leader'),
                'suspicion': st.get('suspicion'),
                'navigation_target': st.get('navigation_target'),
                'predicted_target': st.get('predicted_target'),
                'commanded_target': st.get('commanded_target'),
                'coverage_count': st.get('coverage_count'),
            }
        state_line = json.dumps(record) + '\n'
        with self._rec_lock:
            self._rec_queue.append((frame.copy(), state_line))

    def stop_recording(self):
        if hasattr(self, '_rec_stop'):
            self._rec_stop.set()
            if hasattr(self, '_rec_thread'):
                self._rec_thread.join(timeout=3)
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        if self.state_log_file:
            self.state_log_file.close()
            self.state_log_file = None
        self.recording = False
        print(f"[REC] Stopped — saved to {self.trial_dir}")

    # Two height corrections — homography maps the wall-top plane (152mm).
    # 1. Arena contour: wall-top → floor (0mm), scale outward ~7.5%
    # 2. Robot positions: wall-top → robot tag height (80mm), scale outward ~3.5%
    CAM_H = 2190             # mm — camera above arena floor
    CORNER_H = 152           # mm — corner tags on 6-inch walls
    ROBOT_TAG_H = 80         # mm — robot ArUco tags are 8 cm up
    CAMERA_CENTER_X = 1000   # arena X directly below camera
    CAMERA_CENTER_Y = -40    # arena Y directly below camera (750 - 790)
    FLOOR_SCALE = CAM_H / (CAM_H - CORNER_H)                    # ~1.075 for arena overlay
    ROBOT_SCALE = (CAM_H - ROBOT_TAG_H) / (CAM_H - CORNER_H)   # ~1.035 for robot positions

    def pixel_to_arena(self, px: float, py: float) -> Tuple[float, float]:
        """Convert pixel coords to arena coords (mm), corrected for floor plane."""
        if self.homography is not None:
            pt = np.array([[[px, py]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(pt, self.homography)
            x_mm = float(transformed[0][0][0])
            y_mm = float(transformed[0][0][1])

            # Correct from wall-top plane to robot tag plane
            x_mm = self.CAMERA_CENTER_X + (x_mm - self.CAMERA_CENTER_X) * self.ROBOT_SCALE
            y_mm = self.CAMERA_CENTER_Y + (y_mm - self.CAMERA_CENTER_Y) * self.ROBOT_SCALE
            return x_mm, y_mm
        else:
            x_mm = (px - self.origin_x) / self.pixels_per_mm_x
            y_mm = (self.origin_y - py) / self.pixels_per_mm_y
            return x_mm, y_mm
    
    def update_homography(self, corners: dict, ids: np.ndarray, all_corners: list):
        """
        Update homography from 4 corner ArUco markers.
        Corner markers should be placed at arena corners with IDs 100-103:
          100 = bottom-left (0, 0)
          101 = bottom-right (ARENA_WIDTH, 0)
          102 = top-right (ARENA_WIDTH, ARENA_HEIGHT)
          103 = top-left (0, ARENA_HEIGHT)
        """
        if ids is None:
            return
        
        ids_flat = ids.flatten()
        
        # Check if all 4 corner markers are visible
        corner_pixels = {}
        for i, marker_id in enumerate(ids_flat):
            if marker_id in CORNER_MARKER_IDS:
                # Get center of this corner marker
                corner = all_corners[i][0]
                cx = np.mean(corner[:, 0])
                cy = np.mean(corner[:, 1])
                corner_pixels[marker_id] = (cx, cy)
        
        if len(corner_pixels) < 4:
            # Not all corners visible, keep previous homography
            return
        
        # Source points (pixel coordinates of corner markers)
        src_pts = np.array([
            corner_pixels[40],  # bottom-left
            corner_pixels[41],  # bottom-right
            corner_pixels[42],  # top-right
            corner_pixels[43],  # top-left
        ], dtype=np.float32)
        
        # Destination points (real-world coords of corner TAGS in mm).
        # Tags sit on the outer blue-tape border, offset in X only:
        #   BL(40) = 11.5 cm left of arena origin
        #   BR(41) = 211  cm right of origin (11 cm past right edge)
        #   TR(42) = 11   cm right of right edge, level with y=1500
        #   TL(43) = 11.3 cm left of origin,      level with y=1500
        dst_pts = np.array([
            [-115,            0],   # BL — 11.5 cm left of origin
            [2110,            0],   # BR — 211 cm right of origin
            [2110, ARENA_HEIGHT_MM],   # TR — 11 cm past right edge
            [-113, ARENA_HEIGHT_MM],   # TL — 11.3 cm left of origin
        ], dtype=np.float32)
        
        # Compute homography (pixel→arena) and its inverse (arena→pixel)
        self.homography, _ = cv2.findHomography(src_pts, dst_pts)
        self.inv_homography = np.linalg.inv(self.homography)
        self._cell_polys = []  # invalidate cache so overlay rebuilds
        
        if self.frame_count % 100 == 0:
            print(f"[HOMOGRAPHY] Updated from corner markers")
    
    def detect_and_send(self, frame: np.ndarray) -> Dict[int, dict]:
        """Detect markers and send positions to each robot"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        poses = {}
        
        if ids is not None:
            # Update homography from corner markers (if visible)
            if self.use_homography:
                self.update_homography({}, ids, corners)
            for i, marker_id in enumerate(ids.flatten()):
                # Skip corner calibration markers
                if marker_id in CORNER_MARKER_IDS:
                    continue
                
                corner = corners[i][0]
                
                # Center
                cx = np.mean(corner[:, 0])
                cy = np.mean(corner[:, 1])
                
                # Orientation: "forward" = top of marker (bottom-mid → top-mid)
                top_mid = (corner[0] + corner[1]) / 2
                bot_mid = (corner[2] + corner[3]) / 2
                dx = top_mid[0] - bot_mid[0]
                dy = top_mid[1] - bot_mid[1]
                theta = np.arctan2(-dy, dx)
                
                # Convert to arena coords
                x_mm, y_mm = self.pixel_to_arena(cx, cy)
                
                pose = {
                    'robot_id': int(marker_id),
                    'x': x_mm,
                    'y': y_mm,
                    'theta': theta,
                    'timestamp': time.time(),
                    'px': int(cx),
                    'py': int(cy),
                }
                poses[int(marker_id)] = pose
                
                # Send to this robot only
                self.send_position(int(marker_id), pose)
        
        return poses
    
    def send_position(self, robot_id: int, pose: dict):
        """Send position to specific robot.
        
        All robots listen on UDP_PORT (5100).  Message includes robot_id
        so each robot extracts only its own position.
        """
        msg = {
            'type': 'position',
            'robot_id': robot_id,
            'x': float(pose['x']),
            'y': float(pose['y']),
            'theta': float(pose['theta']),
            'timestamp': float(pose['timestamp'])
        }
        data = json.dumps(msg).encode()
        
        target_ip = ROBOT_IPS.get(robot_id)
        if target_ip:
            self.socket.sendto(data, (target_ip, UDP_PORT))
        else:
            self.socket.sendto(data, (BROADCAST_IP, UDP_PORT))
    
    def arena_to_pixel(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """Convert floor-level arena coords to pixel coords for overlay drawing.
        Undoes the floor correction (FLOOR_SCALE) to get wall-top coords,
        then uses the inverse homography to get pixel coords."""
        if self.inv_homography is not None:
            wx = self.CAMERA_CENTER_X + (x_mm - self.CAMERA_CENTER_X) / self.FLOOR_SCALE
            wy = self.CAMERA_CENTER_Y + (y_mm - self.CAMERA_CENTER_Y) / self.FLOOR_SCALE
            pt = np.array([[[wx, wy]]], dtype=np.float32)
            px_pt = cv2.perspectiveTransform(pt, self.inv_homography)
            return int(px_pt[0][0][0]), int(px_pt[0][0][1])
        return int(x_mm * self.pixels_per_mm_x), int(self.origin_y - y_mm * self.pixels_per_mm_y)

    def _is_wall_cell(self, cx: int, cy: int) -> bool:
        """Mirror of robot's _is_wall_cell for overlay rendering."""
        OUTER_WALL_MARGIN = 115  # swept radius (100) + 15mm gap
        DIVIDER_MARGIN = 125
        BODY_RADIUS = 77
        CELL_SIZE = 125
        INTERIOR_WALL_X_LEFT = 1000
        INTERIOR_WALL_X_RIGHT = 1036
        INTERIOR_WALL_Y_END = 1200

        x = cx * CELL_SIZE + CELL_SIZE / 2
        y = cy * CELL_SIZE + CELL_SIZE / 2
        if x < OUTER_WALL_MARGIN or x > ARENA_WIDTH_MM - OUTER_WALL_MARGIN:
            return True
        if y < OUTER_WALL_MARGIN or y > ARENA_HEIGHT_MM - OUTER_WALL_MARGIN:
            return True
        if y < INTERIOR_WALL_Y_END and (
            INTERIOR_WALL_X_LEFT - DIVIDER_MARGIN < x < INTERIOR_WALL_X_RIGHT + DIVIDER_MARGIN):
            return True
        if INTERIOR_WALL_X_LEFT - BODY_RADIUS < x < INTERIOR_WALL_X_RIGHT + BODY_RADIUS:
            return True
        return False

    def _build_cell_cache(self):
        """Pre-compute cell pixel corners and wall status (called once when
        homography is established, avoids 768 perspective transforms per frame)."""
        CELL_SIZE = 125
        cells_x = int(ARENA_WIDTH_MM / CELL_SIZE)
        cells_y = int(ARENA_HEIGHT_MM / CELL_SIZE)
        self._cell_polys = []
        self._cell_wall = []
        self._num_explorable = 0
        for cy in range(cells_y):
            for cx in range(cells_x):
                x0, y0 = cx * CELL_SIZE, cy * CELL_SIZE
                corners = [(x0, y0), (x0 + CELL_SIZE, y0),
                           (x0 + CELL_SIZE, y0 + CELL_SIZE), (x0, y0 + CELL_SIZE)]
                pts = np.array([self.arena_to_pixel(ax, ay) for ax, ay in corners],
                               dtype=np.int32)
                is_wall = self._is_wall_cell(cx, cy)
                self._cell_polys.append(((cx, cy), pts, is_wall))
                self._cell_wall.append(is_wall)
                if not is_wall:
                    self._num_explorable += 1

    def draw_exploration_overlay(self, frame: np.ndarray):
        """Draw live exploration coverage: green=visited, dark=unexplored, gray=wall."""
        if self.inv_homography is None:
            return

        if not hasattr(self, '_cell_polys') or not self._cell_polys:
            self._build_cell_cache()

        with self._state_lock:
            states = dict(self.robot_states)

        for st in states.values():
            for cell in st.get('visited_cells', []):
                if isinstance(cell, (list, tuple)) and len(cell) == 2:
                    self._visited_cells.add((int(cell[0]), int(cell[1])))
        visited = self._visited_cells

        overlay = frame.copy()
        for (cx, cy), pts, is_wall in self._cell_polys:
            if is_wall:
                cv2.fillConvexPoly(overlay, pts, (60, 60, 60))
            elif (cx, cy) in visited:
                cv2.fillConvexPoly(overlay, pts, (0, 140, 0))
            else:
                cv2.fillConvexPoly(overlay, pts, (40, 20, 20))

        cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)

        pct = len(visited) / max(1, self._num_explorable) * 100
        h, w = frame.shape[:2]
        cov_text = f"Coverage: {len(visited)} cells ({pct:.0f}%)"
        cov_size = cv2.getTextSize(cov_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.putText(frame, cov_text, ((w - cov_size[0]) // 2, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    def draw_arena_contour(self, frame: np.ndarray):
        """Draw the arena model as the robots see it: walls, margin, grid."""
        if self.inv_homography is None:
            return

        WALL_MARGIN = 115  # OUTER_WALL_MARGIN (swept radius + gap)
        DIVIDER_MARGIN = 125
        CELL_SIZE = 125
        INTERIOR_WALL_X_LEFT = 1000
        INTERIOR_WALL_X_RIGHT = 1036
        INTERIOR_WALL_Y_END = 1200
        cells_x = int(ARENA_WIDTH_MM / CELL_SIZE)
        cells_y = int(ARENA_HEIGHT_MM / CELL_SIZE)

        overlay = frame.copy()

        # Arena boundary (green)
        arena_pts = [(0, 0), (ARENA_WIDTH_MM, 0),
                     (ARENA_WIDTH_MM, ARENA_HEIGHT_MM), (0, ARENA_HEIGHT_MM)]
        pts = [self.arena_to_pixel(x, y) for x, y in arena_pts]
        for i in range(4):
            cv2.line(overlay, pts[i], pts[(i + 1) % 4], (0, 255, 0), 2)

        # Interior wall (green) - draw both faces
        cv2.line(overlay,
                 self.arena_to_pixel(INTERIOR_WALL_X_LEFT, 0),
                 self.arena_to_pixel(INTERIOR_WALL_X_LEFT, INTERIOR_WALL_Y_END),
                 (0, 255, 0), 2)
        cv2.line(overlay,
                 self.arena_to_pixel(INTERIOR_WALL_X_RIGHT, 0),
                 self.arena_to_pixel(INTERIOR_WALL_X_RIGHT, INTERIOR_WALL_Y_END),
                 (0, 255, 0), 2)

        # WALL_MARGIN zone (yellow)
        margin_pts = [(WALL_MARGIN, WALL_MARGIN),
                      (ARENA_WIDTH_MM - WALL_MARGIN, WALL_MARGIN),
                      (ARENA_WIDTH_MM - WALL_MARGIN, ARENA_HEIGHT_MM - WALL_MARGIN),
                      (WALL_MARGIN, ARENA_HEIGHT_MM - WALL_MARGIN)]
        mpts = [self.arena_to_pixel(x, y) for x, y in margin_pts]
        for i in range(4):
            cv2.line(overlay, mpts[i], mpts[(i + 1) % 4], (0, 200, 255), 1)

        # Interior wall margins (yellow — uses DIVIDER_MARGIN from each face)
        cv2.line(overlay,
                 self.arena_to_pixel(INTERIOR_WALL_X_LEFT - DIVIDER_MARGIN, 0),
                 self.arena_to_pixel(INTERIOR_WALL_X_LEFT - DIVIDER_MARGIN, INTERIOR_WALL_Y_END),
                 (0, 200, 255), 1)
        cv2.line(overlay,
                 self.arena_to_pixel(INTERIOR_WALL_X_RIGHT + DIVIDER_MARGIN, 0),
                 self.arena_to_pixel(INTERIOR_WALL_X_RIGHT + DIVIDER_MARGIN, INTERIOR_WALL_Y_END),
                 (0, 200, 255), 1)

        # Grid cells (gray)
        for cx in range(1, cells_x):
            cv2.line(overlay,
                     self.arena_to_pixel(cx * CELL_SIZE, 0),
                     self.arena_to_pixel(cx * CELL_SIZE, ARENA_HEIGHT_MM),
                     (100, 100, 100), 1)
        for cy in range(1, cells_y):
            cv2.line(overlay,
                     self.arena_to_pixel(0, cy * CELL_SIZE),
                     self.arena_to_pixel(ARENA_WIDTH_MM, cy * CELL_SIZE),
                     (100, 100, 100), 1)

        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    def draw_overlay(self, frame: np.ndarray, poses: Dict[int, dict]):
        """Draw detection overlay with navigation vectors and trust indicators."""
        with self._state_lock:
            states = dict(self.robot_states)

        for robot_id, pose in poses.items():
            px = pose['px']
            py = pose['py']
            st = states.get(robot_id, {})

            is_leader = (st.get('state') == 'leader')
            trust = st.get('trust_in_leader')
            suspicion = st.get('suspicion', 0)

            # --- Ring color: green=healthy, yellow=suspicious, red=distrusting ---
            if trust is not None and trust < 0.5:
                ring_color = (0, 0, 255)       # red — lost trust
            elif suspicion and suspicion > 0.4:
                ring_color = (0, 180, 255)     # orange — suspicious
            else:
                ring_color = (0, 255, 0)       # green — normal

            ring_thick = 3 if is_leader else 2
            cv2.circle(frame, (px, py), 22, ring_color, ring_thick)
            if is_leader:
                cv2.circle(frame, (px, py), 28, (0, 255, 255), 1)

            # --- Heading arrow (green, short) ---
            arrow_len = 40
            ax = int(px + arrow_len * np.cos(-pose['theta']))
            ay = int(py + arrow_len * np.sin(-pose['theta']))
            cv2.arrowedLine(frame, (px, py), (ax, ay), (0, 255, 0), 2)

            # --- Navigation target arrow (cyan, longer) ---
            nav_tgt = st.get('navigation_target')
            if nav_tgt and self.inv_homography is not None:
                tpx, tpy = self.arena_to_pixel(nav_tgt[0], nav_tgt[1])
                dx, dy = tpx - px, tpy - py
                dist = math.sqrt(dx * dx + dy * dy)
                if dist > 5:
                    max_arrow = 80
                    scale = min(max_arrow / dist, 1.0)
                    epx = int(px + dx * scale)
                    epy = int(py + dy * scale)
                    cv2.arrowedLine(frame, (px, py), (epx, epy), (255, 200, 0), 2, tipLength=0.25)

            # --- Bad command indicator (red arrow for commanded, green for predicted) ---
            cmd_tgt = st.get('commanded_target')
            pred_tgt = st.get('predicted_target')
            if cmd_tgt and pred_tgt and self.inv_homography is not None:
                cdx = cmd_tgt[0] - pose['x']
                cdy = cmd_tgt[1] - pose['y']
                pdx = pred_tgt[0] - pose['x']
                pdy = pred_tgt[1] - pose['y']
                cmd_len = math.sqrt(cdx * cdx + cdy * cdy)
                pred_len = math.sqrt(pdx * pdx + pdy * pdy)
                if cmd_len > 1 and pred_len > 1:
                    dot = (cdx * pdx + cdy * pdy) / (cmd_len * pred_len)
                    dot = max(-1.0, min(1.0, dot))
                    angle_diff = math.degrees(math.acos(dot))
                    if angle_diff > 30:
                        # Significant divergence — show red (commanded) vs green (predicted)
                        cpx, cpy = self.arena_to_pixel(cmd_tgt[0], cmd_tgt[1])
                        ddx, ddy = cpx - px, cpy - py
                        cd = math.sqrt(ddx * ddx + ddy * ddy)
                        if cd > 5:
                            sc = min(90 / cd, 1.0)
                            cv2.arrowedLine(frame, (px, py),
                                            (int(px + ddx * sc), int(py + ddy * sc)),
                                            (0, 0, 255), 3, tipLength=0.3)
                        ppx, ppy = self.arena_to_pixel(pred_tgt[0], pred_tgt[1])
                        ddx2, ddy2 = ppx - px, ppy - py
                        pd = math.sqrt(ddx2 * ddx2 + ddy2 * ddy2)
                        if pd > 5:
                            sc2 = min(90 / pd, 1.0)
                            cv2.arrowedLine(frame, (px, py),
                                            (int(px + ddx2 * sc2), int(py + ddy2 * sc2)),
                                            (0, 255, 100), 3, tipLength=0.3)

            # --- Label ---
            label = f"{'L' if is_leader else 'R'}{robot_id}"
            cv2.putText(frame, label, (px - 15, py - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, ring_color, 2)
            trust_str = f"T:{trust:.2f}" if trust is not None else ""
            cv2.putText(frame, f"({pose['x']:.0f},{pose['y']:.0f}) {trust_str}",
                       (px - 40, py + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # FPS + recording indicator (centered at top)
        h, w = frame.shape[:2]
        rec_tag = " [REC]" if self.recording else ""
        fps_text = f"FPS: {self.fps:.1f} | Robots: {len(poses)}{rec_tag}"
        fps_color = (0, 0, 255) if self.recording else (0, 255, 255)
        fps_size = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        cv2.putText(frame, fps_text, ((w - fps_size[0]) // 2, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, fps_color, 2)

        mode_text = "HOMOGRAPHY" if self.homography is not None else "LINEAR (no corners)"
        mode_display = f"Mode: {mode_text}"
        mode_color = (0, 255, 0) if self.homography is not None else (0, 165, 255)
        mode_size = cv2.getTextSize(mode_display, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.putText(frame, mode_display, ((w - mode_size[0]) // 2, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)
    
    def calibrate(self):
        """Interactive calibration"""
        print("\nCalibration: Click 3 corners (bottom-left, bottom-right, top-right)")
        print("Press 'c' to confirm, 'r' to reset, 'q' to skip\n")
        
        corners = []
        
        def mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 3:
                corners.append((x, y))
                print(f"Corner {len(corners)}: ({x}, {y})")
        
        cv2.namedWindow('Calibration')
        cv2.setMouseCallback('Calibration', mouse_cb)
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Draw corners
            colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
            labels = ["Origin", "X-axis", "Top-right"]
            for i, (x, y) in enumerate(corners):
                cv2.circle(frame, (x, y), 10, colors[i], -1)
                cv2.putText(frame, labels[i], (x + 15, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2)
            
            if len(corners) >= 2:
                cv2.line(frame, corners[0], corners[1], (0, 255, 0), 2)
            if len(corners) >= 3:
                cv2.line(frame, corners[1], corners[2], (0, 255, 0), 2)
            
            cv2.putText(frame, f"Corners: {len(corners)}/3", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Calibration', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c') and len(corners) == 3:
                self.origin_x = corners[0][0]
                self.origin_y = corners[0][1]
                
                dx = corners[1][0] - corners[0][0]
                dy = corners[1][1] - corners[0][1]
                x_dist = np.sqrt(dx*dx + dy*dy)
                self.pixels_per_mm_x = x_dist / ARENA_WIDTH_MM
                
                dx = corners[2][0] - corners[1][0]
                dy = corners[2][1] - corners[1][1]
                y_dist = np.sqrt(dx*dx + dy*dy)
                self.pixels_per_mm_y = y_dist / ARENA_HEIGHT_MM
                
                print(f"\nCalibration applied!")
                print(f"Origin: ({self.origin_x}, {self.origin_y})")
                print(f"Scale: {self.pixels_per_mm_x:.4f}, {self.pixels_per_mm_y:.4f} px/mm\n")
                break
            elif key == ord('r'):
                corners = []
            elif key == ord('q'):
                break
        
        cv2.destroyWindow('Calibration')
    
    def run(self):
        """Main loop"""
        self.running = True
        interval = 1.0 / POSITION_RATE

        print("Running... 'q'=quit 'r'=record 'c'=calibrate 'v'=video 'e'=exploration\n")

        while self.running:
            start = time.time()

            ret, frame = self.cap.read()
            if not ret:
                break

            poses = self.detect_and_send(frame)

            # FPS calc
            self.frame_count += 1
            if time.time() - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (time.time() - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = time.time()

            if self.show_video:
                if self.show_exploration:
                    self.draw_exploration_overlay(frame)
                self.draw_arena_contour(frame)
                self.draw_overlay(frame, poses)
                cv2.imshow('Position Server', frame)

            # Record frame + state (non-blocking, queued to background thread)
            if self.recording:
                self._enqueue_frame(frame, poses)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running = False
            elif key == ord('r'):
                if not self.recording:
                    self.start_recording(frame.shape)
                else:
                    self.stop_recording()
            elif key == ord('c'):
                self.calibrate()
            elif key == ord('v'):
                self.show_video = not self.show_video
                if not self.show_video:
                    cv2.destroyAllWindows()
            elif key == ord('e'):
                self.show_exploration = not self.show_exploration
                print(f"Exploration overlay: {'ON' if self.show_exploration else 'OFF'}")

            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)

        if self.recording:
            self.stop_recording()
        self.cap.release()
        cv2.destroyAllWindows()
        print("Stopped.")

# ============== Entry ==============
if __name__ == "__main__":
    import sys
    
    camera_id = CAMERA_ID
    if len(sys.argv) > 1:
        camera_id = int(sys.argv[1])
    
    server = PositionServer(camera_id)
    
    if '--calibrate' in sys.argv:
        server.calibrate()
    
    server.run()
