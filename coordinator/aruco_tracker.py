#!/usr/bin/env python3
"""
ArUco Marker Tracker for REIP Robot Fleet
Detects markers, extracts pose, broadcasts positions.

Usage: python aruco_tracker.py [--camera 0] [--show]

Requirements:
    pip install opencv-python opencv-contrib-python numpy
"""

import cv2
import numpy as np
import socket
import json
import time
import argparse
import math

# ============================================================================
# CONFIGURATION
# ============================================================================

POSITION_PORT = 5003      # Broadcast positions to coordinator
BROADCAST_RATE = 30       # Hz

# ArUco configuration
ARUCO_DICT = cv2.aruco.DICT_4X4_50  # 4x4 markers, IDs 0-49
MARKER_SIZE_MM = 50       # Physical marker size in mm (adjust to your markers)

# Camera calibration (default values - calibrate for accuracy!)
# These are approximate for a typical webcam
CAMERA_MATRIX = np.array([
    [800, 0, 320],
    [0, 800, 240],
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.zeros((4, 1))  # Assuming no distortion

# Coordinate transform: camera frame to world frame
# Adjust based on your camera mounting
CAMERA_HEIGHT_MM = 1500   # Camera height above table
FLIP_X = False
FLIP_Y = True             # Usually Y is inverted


# ============================================================================
# TRACKER
# ============================================================================

class ArUcoTracker:
    def __init__(self, camera_id=0, show_preview=False):
        self.show_preview = show_preview
        
        # Initialize camera
        print(f"[Tracker] Opening camera {camera_id}...")
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_id}")
        
        # Set resolution (adjust as needed)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # State
        self.positions = {}  # id -> (x, y, theta, timestamp)
        
        print("[Tracker] Ready!")
    
    def detect_markers(self, frame):
        """Detect ArUco markers and extract poses."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        detections = {}
        
        if ids is not None:
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE_MM, CAMERA_MATRIX, DIST_COEFFS
            )
            
            for i, marker_id in enumerate(ids.flatten()):
                # Get translation (position)
                tvec = tvecs[i][0]
                x_mm = tvec[0]
                y_mm = tvec[1]
                
                # Apply coordinate transform
                if FLIP_X:
                    x_mm = -x_mm
                if FLIP_Y:
                    y_mm = -y_mm
                
                # Get rotation (heading)
                rvec = rvecs[i][0]
                rmat, _ = cv2.Rodrigues(rvec)
                # Extract yaw (rotation around Z axis)
                theta = math.atan2(rmat[1, 0], rmat[0, 0])
                
                detections[int(marker_id)] = {
                    'x': float(x_mm),
                    'y': float(y_mm),
                    'theta': float(theta),
                    'corners': corners[i][0].tolist(),
                }
                
                # Draw on frame if preview enabled
                if self.show_preview:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, 
                                     rvecs[i], tvecs[i], MARKER_SIZE_MM * 0.5)
        
        return detections, frame
    
    def broadcast_positions(self, detections):
        """Send positions via UDP."""
        msg = {
            'ts': time.time(),
            'robots': {}
        }
        
        for marker_id, data in detections.items():
            msg['robots'][marker_id] = {
                'x': data['x'],
                'y': data['y'],
                'theta': data['theta'],
            }
        
        data = json.dumps(msg).encode()
        self.sock.sendto(data, ('<broadcast>', POSITION_PORT))
    
    def run(self):
        """Main tracking loop."""
        print("[Tracker] Running... Press 'q' to quit")
        
        frame_interval = 1.0 / BROADCAST_RATE
        last_broadcast = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("[Tracker] Failed to read frame")
                    continue
                
                # Detect markers
                detections, annotated = self.detect_markers(frame)
                
                # Update positions
                now = time.time()
                for marker_id, data in detections.items():
                    self.positions[marker_id] = (data['x'], data['y'], data['theta'], now)
                
                # Broadcast at fixed rate
                if now - last_broadcast >= frame_interval:
                    self.broadcast_positions(detections)
                    last_broadcast = now
                    
                    # Status
                    ids = list(detections.keys())
                    print(f"\r[Tracker] Detected: {ids}    ", end="")
                
                # Show preview
                if self.show_preview:
                    # Add status text
                    cv2.putText(annotated, f"Detected: {len(detections)}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    for marker_id, data in detections.items():
                        x, y, theta = data['x'], data['y'], data['theta']
                        text = f"ID{marker_id}: ({x:.0f}, {y:.0f}, {math.degrees(theta):.0f}deg)"
                        cv2.putText(annotated, text, (10, 60 + marker_id * 25),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    cv2.imshow('ArUco Tracker', annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
        except KeyboardInterrupt:
            print("\n[Tracker] Shutting down...")
        finally:
            self.cap.release()
            self.sock.close()
            if self.show_preview:
                cv2.destroyAllWindows()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='ArUco Tracker for REIP')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID')
    parser.add_argument('--show', action='store_true', help='Show preview window')
    args = parser.parse_args()
    
    tracker = ArUcoTracker(camera_id=args.camera, show_preview=args.show)
    tracker.run()


if __name__ == "__main__":
    main()
