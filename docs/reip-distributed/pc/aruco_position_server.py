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
from dataclasses import dataclass
from typing import Dict, Tuple

# ============== Configuration ==============
CAMERA_ID = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# ArUco
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Arena (calibrate these to your pool table)
ARENA_WIDTH_MM = 2000   # ~2m pool table width
ARENA_HEIGHT_MM = 1000  # ~1m pool table height

# Network
UDP_PORT = 5003  # Position updates to robots
BROADCAST_IP = "255.255.255.255"
POSITION_RATE = 30  # Hz

# Robot IPs (will broadcast, robots filter by ID)
# Or can unicast if you set these
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
        
        # Camera
        print(f"Opening camera {camera_id}...")
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_id}")
        
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Camera: {int(actual_w)}x{int(actual_h)}")
        
        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Calibration (pixels to mm) - default assumes camera sees full arena
        self.pixels_per_mm_x = actual_w / ARENA_WIDTH_MM
        self.pixels_per_mm_y = actual_h / ARENA_HEIGHT_MM
        self.origin_x = 0
        self.origin_y = actual_h
        
        # Network
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # State
        self.running = False
        self.show_video = True
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()
        
        print("Ready!\n")
    
    def pixel_to_arena(self, px: float, py: float) -> Tuple[float, float]:
        """Convert pixel coords to arena coords (mm)"""
        x_mm = (px - self.origin_x) / self.pixels_per_mm_x
        y_mm = (self.origin_y - py) / self.pixels_per_mm_y
        return x_mm, y_mm
    
    def detect_and_send(self, frame: np.ndarray) -> Dict[int, dict]:
        """Detect markers and send positions to each robot"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        poses = {}
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]
                
                # Center
                cx = np.mean(corner[:, 0])
                cy = np.mean(corner[:, 1])
                
                # Orientation
                dx = corner[1][0] - corner[0][0]
                dy = corner[1][1] - corner[0][1]
                theta = np.arctan2(-dy, dx)
                
                # Convert to arena coords
                x_mm, y_mm = self.pixel_to_arena(cx, cy)
                
                pose = {
                    'robot_id': int(marker_id),
                    'x': x_mm,
                    'y': y_mm,
                    'theta': theta,
                    'timestamp': time.time()
                }
                poses[int(marker_id)] = pose
                
                # Send to this robot only
                self.send_position(int(marker_id), pose)
        
        return poses
    
    def send_position(self, robot_id: int, pose: dict):
        """Send position to specific robot"""
        msg = {
            'type': 'position',
            'robot_id': robot_id,
            'x': pose['x'],
            'y': pose['y'],
            'theta': pose['theta'],
            'timestamp': pose['timestamp']
        }
        data = json.dumps(msg).encode()
        
        # Unicast if IP known, otherwise broadcast (robot filters by ID)
        target_ip = ROBOT_IPS.get(robot_id)
        if target_ip:
            self.socket.sendto(data, (target_ip, UDP_PORT))
        else:
            self.socket.sendto(data, (BROADCAST_IP, UDP_PORT))
    
    def draw_overlay(self, frame: np.ndarray, poses: Dict[int, dict]):
        """Draw detection overlay"""
        for robot_id, pose in poses.items():
            px = int(self.origin_x + pose['x'] * self.pixels_per_mm_x)
            py = int(self.origin_y - pose['y'] * self.pixels_per_mm_y)
            
            # Circle
            cv2.circle(frame, (px, py), 20, (0, 255, 0), 2)
            
            # Arrow for heading
            arrow_len = 40
            ax = int(px + arrow_len * np.cos(-pose['theta']))
            ay = int(py + arrow_len * np.sin(-pose['theta']))
            cv2.arrowedLine(frame, (px, py), (ax, ay), (0, 255, 0), 2)
            
            # Label
            cv2.putText(frame, f"R{robot_id}", (px - 15, py - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"({pose['x']:.0f},{pose['y']:.0f})", (px - 30, py + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # FPS
        cv2.putText(frame, f"FPS: {self.fps:.1f} | Robots: {len(poses)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Mode indicator
        cv2.putText(frame, "GPS SATELLITE MODE", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
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
        
        print("Running... Press 'q' to quit, 'c' to calibrate, 'v' to toggle video\n")
        
        while self.running:
            start = time.time()
            
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Detect and send
            poses = self.detect_and_send(frame)
            
            # FPS calc
            self.frame_count += 1
            if time.time() - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (time.time() - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = time.time()
            
            # Display
            if self.show_video:
                self.draw_overlay(frame, poses)
                cv2.imshow('Position Server', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running = False
            elif key == ord('c'):
                self.calibrate()
            elif key == ord('v'):
                self.show_video = not self.show_video
                if not self.show_video:
                    cv2.destroyAllWindows()
            
            # Rate limit
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)
        
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
