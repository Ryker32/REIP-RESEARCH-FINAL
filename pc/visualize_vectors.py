#!/usr/bin/env python3
"""
Vector Overlay Visualization for REIP Experiments

Post-processes robot logs + video to create visualization showing:
- Green arrow: Robot's predicted movement vector (local MPC optimal)
- Red arrow: Leader's commanded vector
- Yellow contour: Trust wavering (suspicion building)
- Robot trail showing visited cells

Usage:
  python visualize_vectors.py --logs logs/ --video experiment.mp4 --output overlay.mp4
  python visualize_vectors.py --logs logs/  # No video, just generate frames

For WSSEF presentation: Compare WITH REIP vs WITHOUT REIP side-by-side
"""

import cv2
import json
import numpy as np
import argparse
import os
from glob import glob
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# ============== Configuration ==============
CELL_SIZE = 125  # mm - must match reip_node.py
ARENA_WIDTH = 2000
ARENA_HEIGHT = 1500

# Colors (BGR for OpenCV)
COLOR_PREDICTED = (0, 255, 0)     # Green - robot's local optimal
COLOR_COMMANDED = (0, 0, 255)     # Red - leader's command
COLOR_ROBOT = (255, 200, 0)       # Cyan - robot body
COLOR_LEADER = (0, 255, 255)      # Yellow - leader robot
COLOR_TRUST_LOW = (0, 255, 255)   # Yellow contour - wavering trust
COLOR_TRUST_VERY_LOW = (0, 165, 255)  # Orange contour - about to impeach
COLOR_VISITED = (100, 100, 100)   # Gray - visited cells
COLOR_TRAIL = (180, 180, 180)     # Light gray - movement trail

# Trust thresholds for visualization
TRUST_WAVERING = 0.7
TRUST_CRITICAL = 0.4

# ============== Data Structures ==============
@dataclass
class RobotState:
    robot_id: int
    x: float
    y: float
    theta: float
    state: str
    leader_id: Optional[int]
    trust_in_leader: float
    suspicion: float
    predicted_target: Optional[Tuple[float, float]]
    commanded_target: Optional[Tuple[float, float]]
    fault: Optional[str]

# ============== Coordinate Transform ==============
class CoordinateTransform:
    """Transform arena coordinates to video/image coordinates"""
    
    def __init__(self, img_width: int, img_height: int, margin: int = 50):
        self.img_width = img_width
        self.img_height = img_height
        self.margin = margin
        
        # Scale to fit arena in image with margin
        usable_width = img_width - 2 * margin
        usable_height = img_height - 2 * margin
        
        self.scale = min(usable_width / ARENA_WIDTH, usable_height / ARENA_HEIGHT)
        
        # Center the arena
        scaled_width = ARENA_WIDTH * self.scale
        scaled_height = ARENA_HEIGHT * self.scale
        self.offset_x = (img_width - scaled_width) / 2
        self.offset_y = (img_height - scaled_height) / 2
    
    def arena_to_img(self, x: float, y: float) -> Tuple[int, int]:
        """Convert arena coordinates (mm) to image coordinates (pixels)"""
        px = int(self.offset_x + x * self.scale)
        py = int(self.offset_y + (ARENA_HEIGHT - y) * self.scale)  # Flip Y
        return (px, py)
    
    def cell_to_img(self, cx: int, cy: int) -> Tuple[int, int, int, int]:
        """Get cell bounding box in image coordinates"""
        x1 = int(self.offset_x + cx * CELL_SIZE * self.scale)
        y1 = int(self.offset_y + (ARENA_HEIGHT - (cy + 1) * CELL_SIZE) * self.scale)
        x2 = int(self.offset_x + (cx + 1) * CELL_SIZE * self.scale)
        y2 = int(self.offset_y + (ARENA_HEIGHT - cy * CELL_SIZE) * self.scale)
        return (x1, y1, x2, y2)

# ============== Visualization ==============
class VectorVisualizer:
    def __init__(self, width: int = 1280, height: int = 720):
        self.width = width
        self.height = height
        self.transform = CoordinateTransform(width, height)
        self.trails: Dict[int, List[Tuple[int, int]]] = {}
        self.max_trail_length = 100
    
    def draw_grid(self, frame: np.ndarray, visited_cells: set = None):
        """Draw grid overlay"""
        if visited_cells is None:
            visited_cells = set()
        
        # Draw visited cells
        for cx in range(int(ARENA_WIDTH / CELL_SIZE)):
            for cy in range(int(ARENA_HEIGHT / CELL_SIZE)):
                if (cx, cy) in visited_cells:
                    x1, y1, x2, y2 = self.transform.cell_to_img(cx, cy)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), COLOR_VISITED, -1)
        
        # Draw grid lines
        for cx in range(int(ARENA_WIDTH / CELL_SIZE) + 1):
            x = int(self.transform.offset_x + cx * CELL_SIZE * self.transform.scale)
            cv2.line(frame, (x, int(self.transform.offset_y)),
                    (x, int(self.transform.offset_y + ARENA_HEIGHT * self.transform.scale)),
                    (50, 50, 50), 1)
        
        for cy in range(int(ARENA_HEIGHT / CELL_SIZE) + 1):
            y = int(self.transform.offset_y + (ARENA_HEIGHT - cy * CELL_SIZE) * self.transform.scale)
            cv2.line(frame, (int(self.transform.offset_x), y),
                    (int(self.transform.offset_x + ARENA_WIDTH * self.transform.scale), y),
                    (50, 50, 50), 1)
    
    def draw_robot(self, frame: np.ndarray, state: RobotState):
        """Draw robot with vectors and trust visualization"""
        pos = self.transform.arena_to_img(state.x, state.y)
        
        # Update trail
        if state.robot_id not in self.trails:
            self.trails[state.robot_id] = []
        self.trails[state.robot_id].append(pos)
        if len(self.trails[state.robot_id]) > self.max_trail_length:
            self.trails[state.robot_id].pop(0)
        
        # Draw trail
        trail = self.trails[state.robot_id]
        for i in range(1, len(trail)):
            alpha = i / len(trail)
            color = tuple(int(c * alpha * 0.5) for c in COLOR_TRAIL)
            cv2.line(frame, trail[i-1], trail[i], color, 2)
        
        # Determine robot color and contour based on trust
        robot_color = COLOR_LEADER if state.state == "leader" else COLOR_ROBOT
        contour_color = None
        contour_thickness = 3
        
        if state.state == "follower":
            if state.trust_in_leader < TRUST_CRITICAL:
                contour_color = COLOR_TRUST_VERY_LOW
                contour_thickness = 5
            elif state.trust_in_leader < TRUST_WAVERING:
                contour_color = COLOR_TRUST_LOW
                contour_thickness = 4
        
        # Draw trust contour (yellow/orange ring for wavering trust)
        if contour_color:
            cv2.circle(frame, pos, 25, contour_color, contour_thickness)
        
        # Draw robot body
        radius = 15
        cv2.circle(frame, pos, radius, robot_color, -1)
        cv2.circle(frame, pos, radius, (255, 255, 255), 2)
        
        # Draw robot heading
        head_len = 25
        head_x = int(pos[0] + head_len * np.cos(state.theta))
        head_y = int(pos[1] - head_len * np.sin(state.theta))  # Flip Y
        cv2.line(frame, pos, (head_x, head_y), (255, 255, 255), 3)
        
        # Draw robot ID
        cv2.putText(frame, str(state.robot_id), 
                   (pos[0] - 5, pos[1] + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # Draw vectors
        arrow_length = 60
        
        # Green: Predicted (local MPC optimal)
        if state.predicted_target:
            pred_pos = self.transform.arena_to_img(state.predicted_target[0], 
                                                    state.predicted_target[1])
            dx = pred_pos[0] - pos[0]
            dy = pred_pos[1] - pos[1]
            dist = np.sqrt(dx*dx + dy*dy)
            if dist > 0:
                dx = int(dx / dist * arrow_length)
                dy = int(dy / dist * arrow_length)
                cv2.arrowedLine(frame, pos, (pos[0] + dx, pos[1] + dy),
                              COLOR_PREDICTED, 3, tipLength=0.3)
        
        # Red: Commanded (leader's assignment)
        if state.commanded_target and state.state == "follower":
            cmd_pos = self.transform.arena_to_img(state.commanded_target[0],
                                                   state.commanded_target[1])
            dx = cmd_pos[0] - pos[0]
            dy = cmd_pos[1] - pos[1]
            dist = np.sqrt(dx*dx + dy*dy)
            if dist > 0:
                # Offset slightly to not overlap with predicted
                offset = 8
                dx = int(dx / dist * arrow_length)
                dy = int(dy / dist * arrow_length)
                cv2.arrowedLine(frame, (pos[0] + offset, pos[1] + offset), 
                              (pos[0] + dx + offset, pos[1] + dy + offset),
                              COLOR_COMMANDED, 3, tipLength=0.3)
    
    def draw_legend(self, frame: np.ndarray):
        """Draw legend explaining colors"""
        x, y = 20, 30
        spacing = 25
        
        cv2.putText(frame, "REIP Visualization", (x, y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        y += spacing + 10
        cv2.arrowedLine(frame, (x, y), (x + 40, y), COLOR_PREDICTED, 3)
        cv2.putText(frame, "Predicted (local optimal)", (x + 50, y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        y += spacing
        cv2.arrowedLine(frame, (x, y), (x + 40, y), COLOR_COMMANDED, 3)
        cv2.putText(frame, "Commanded (leader)", (x + 50, y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        y += spacing
        cv2.circle(frame, (x + 20, y), 12, COLOR_LEADER, -1)
        cv2.putText(frame, "Leader", (x + 50, y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        y += spacing
        cv2.circle(frame, (x + 20, y), 12, COLOR_TRUST_LOW, 3)
        cv2.putText(frame, "Trust wavering", (x + 50, y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        y += spacing
        cv2.circle(frame, (x + 20, y), 12, COLOR_TRUST_VERY_LOW, 4)
        cv2.putText(frame, "Trust critical", (x + 50, y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def draw_stats(self, frame: np.ndarray, states: Dict[int, RobotState], 
                   coverage_pct: float, time_sec: float):
        """Draw stats overlay"""
        x = self.width - 220
        y = 30
        spacing = 22
        
        cv2.putText(frame, f"Time: {time_sec:.1f}s", (x, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        y += spacing
        cv2.putText(frame, f"Coverage: {coverage_pct:.1f}%", (x, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        y += spacing + 5
        cv2.putText(frame, "Robot Trust:", (x, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        for rid, state in sorted(states.items()):
            y += spacing
            trust_str = f"R{rid}: {state.trust_in_leader:.2f}"
            if state.state == "leader":
                trust_str += " (L)"
            color = (255, 255, 255)
            if state.trust_in_leader < TRUST_CRITICAL:
                color = COLOR_TRUST_VERY_LOW
            elif state.trust_in_leader < TRUST_WAVERING:
                color = COLOR_TRUST_LOW
            cv2.putText(frame, trust_str, (x, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    def render_frame(self, states: Dict[int, RobotState], 
                     visited_cells: set = None, time_sec: float = 0) -> np.ndarray:
        """Render a single frame"""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:] = (30, 30, 30)  # Dark background
        
        # Draw grid and visited cells
        self.draw_grid(frame, visited_cells)
        
        # Draw robots
        for state in states.values():
            self.draw_robot(frame, state)
        
        # Draw legend and stats
        self.draw_legend(frame)
        
        total_cells = (ARENA_WIDTH / CELL_SIZE) * (ARENA_HEIGHT / CELL_SIZE)
        coverage = len(visited_cells) / total_cells * 100 if visited_cells else 0
        self.draw_stats(frame, states, coverage, time_sec)
        
        return frame

# ============== Log Processing ==============
def load_logs(log_dir: str) -> Dict[int, List[dict]]:
    """Load all robot logs from directory"""
    logs = {}
    
    for log_file in glob(os.path.join(log_dir, "robot_*.jsonl")):
        robot_id = int(os.path.basename(log_file).split('_')[1])
        logs[robot_id] = []
        
        with open(log_file, 'r') as f:
            for line in f:
                try:
                    record = json.loads(line.strip())
                    logs[robot_id].append(record)
                except:
                    pass
        
        print(f"Loaded {len(logs[robot_id])} records for robot {robot_id}")
    
    return logs

def interpolate_state(logs: Dict[int, List[dict]], t: float) -> Dict[int, RobotState]:
    """Get robot states at time t by interpolation"""
    states = {}
    
    for robot_id, records in logs.items():
        if not records:
            continue
        
        # Find surrounding records
        prev_rec = None
        next_rec = None
        
        for rec in records:
            if rec['t'] <= t:
                prev_rec = rec
            if rec['t'] > t and next_rec is None:
                next_rec = rec
                break
        
        if prev_rec is None:
            prev_rec = records[0]
        if next_rec is None:
            next_rec = prev_rec
        
        # Simple nearest-neighbor (could do linear interp for smoother)
        rec = prev_rec if abs(prev_rec['t'] - t) < abs(next_rec['t'] - t) else next_rec
        
        predicted = None
        if rec.get('predicted_target'):
            predicted = tuple(rec['predicted_target'])
        
        commanded = None
        if rec.get('commanded_target'):
            commanded = tuple(rec['commanded_target'])
        
        states[robot_id] = RobotState(
            robot_id=robot_id,
            x=rec.get('x', 0),
            y=rec.get('y', 0),
            theta=rec.get('theta', 0),
            state=rec.get('state', 'idle'),
            leader_id=rec.get('leader_id'),
            trust_in_leader=rec.get('trust_in_leader', 1.0),
            suspicion=rec.get('suspicion', 0),
            predicted_target=predicted,
            commanded_target=commanded,
            fault=rec.get('fault')
        )
    
    return states

def get_visited_cells(logs: Dict[int, List[dict]], t: float) -> set:
    """Get all visited cells up to time t"""
    visited = set()
    
    for robot_id, records in logs.items():
        for rec in records:
            if rec['t'] > t:
                break
            # Mark cell as visited
            cell = (int(rec.get('x', 0) / CELL_SIZE), 
                   int(rec.get('y', 0) / CELL_SIZE))
            visited.add(cell)
    
    return visited

# ============== Main ==============
def main():
    parser = argparse.ArgumentParser(description="REIP Vector Visualization")
    parser.add_argument('--logs', type=str, required=True, help="Log directory")
    parser.add_argument('--video', type=str, help="Input video (optional)")
    parser.add_argument('--output', type=str, default="reip_overlay.mp4", help="Output video")
    parser.add_argument('--fps', type=float, default=10, help="Output FPS")
    parser.add_argument('--duration', type=float, help="Max duration in seconds")
    parser.add_argument('--start', type=float, default=0, help="Start time offset")
    args = parser.parse_args()
    
    # Load logs
    print(f"Loading logs from {args.logs}...")
    logs = load_logs(args.logs)
    
    if not logs:
        print("No logs found!")
        return
    
    # Determine time range
    all_times = []
    for records in logs.values():
        for rec in records:
            all_times.append(rec['t'])
    
    t_start = min(all_times) + args.start
    t_end = max(all_times)
    if args.duration:
        t_end = min(t_end, t_start + args.duration)
    
    print(f"Time range: {t_start:.1f} to {t_end:.1f} ({t_end - t_start:.1f}s)")
    
    # Create visualizer
    visualizer = VectorVisualizer()
    
    # Video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(args.output, fourcc, args.fps, 
                            (visualizer.width, visualizer.height))
    
    # Render frames
    t = t_start
    frame_count = 0
    
    print("Rendering...")
    while t <= t_end:
        states = interpolate_state(logs, t)
        visited = get_visited_cells(logs, t)
        
        frame = visualizer.render_frame(states, visited, t - t_start)
        writer.write(frame)
        
        t += 1.0 / args.fps
        frame_count += 1
        
        if frame_count % 50 == 0:
            print(f"  Frame {frame_count}, t={t - t_start:.1f}s")
    
    writer.release()
    print(f"\nSaved {frame_count} frames to {args.output}")
    
    # Also save key frames as images for poster
    print("\nGenerating key frames for poster...")
    key_times = [
        t_start,
        t_start + (t_end - t_start) * 0.25,
        t_start + (t_end - t_start) * 0.5,
        t_start + (t_end - t_start) * 0.75,
        t_end - 0.1
    ]
    
    os.makedirs("frames", exist_ok=True)
    for i, kt in enumerate(key_times):
        states = interpolate_state(logs, kt)
        visited = get_visited_cells(logs, kt)
        frame = visualizer.render_frame(states, visited, kt - t_start)
        cv2.imwrite(f"frames/frame_{i+1}.png", frame)
        print(f"  Saved frames/frame_{i+1}.png (t={kt - t_start:.1f}s)")

if __name__ == "__main__":
    main()
