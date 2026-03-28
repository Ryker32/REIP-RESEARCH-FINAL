from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple


TOF_SENSOR_ANGLES_DEG: Dict[str, float] = {
    "right": -75.0,
    "front_right": -37.5,
    "front": 0.0,
    "front_left": 37.5,
    "left": 75.0,
}

SIM_MOTOR_PORT = 5400
SIM_PEER_RELAY_PORT = 5500
SIM_SENSOR_PORT = 5600
POSITION_FRESHNESS_S = 1.0
POSITION_STOP_TIMEOUT_S = 2.0
PEER_TIMEOUT_S = 3.0
LEADER_ACK_STALE_S = 1.5
ASSIGNMENT_STALE_S = 5.0
CAUSALITY_GRACE_S = 0.5
CONTROL_RATE_HZ = 10.0
BROADCAST_RATE_HZ = 5.0


@dataclass(frozen=True)
class ArenaGeometry:
    width_mm: int = 2000
    height_mm: int = 1500
    cell_size_mm: int = 125
    robot_radius_mm: float = 110.0
    body_radius_mm: float = 77.0
    body_half_width_mm: float = 64.0
    body_front_mm: float = 70.0
    swept_radius_mm: float = 100.0
    tof_sensor_offset_mm: float = 70.0
    tof_range_mm: float = 200.0
    avoid_distance_mm: float = 200.0
    critical_distance_mm: float = 100.0
    interior_wall_x_left_mm: float = 1000.0
    interior_wall_x_right_mm: float = 1036.0
    interior_wall_y_end_mm: float = 1200.0
    repulsion_zone_mm: float = 220.0

    @property
    def interior_wall_x_mm(self) -> float:
        return (self.interior_wall_x_left_mm + self.interior_wall_x_right_mm) / 2.0

    @property
    def outer_wall_margin_mm(self) -> float:
        return self.robot_radius_mm

    @property
    def divider_margin_mm(self) -> float:
        return self.body_half_width_mm

    @property
    def divider_tip_clearance_mm(self) -> float:
        return self.swept_radius_mm

    @property
    def cols(self) -> int:
        return int(self.width_mm / self.cell_size_mm)

    @property
    def rows(self) -> int:
        return int(self.height_mm / self.cell_size_mm)

    def cell_to_pos(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        return (
            (cell[0] + 0.5) * self.cell_size_mm,
            (cell[1] + 0.5) * self.cell_size_mm,
        )

    def pos_to_cell(self, x_mm: float, y_mm: float) -> Optional[Tuple[int, int]]:
        cx = int(x_mm / self.cell_size_mm)
        cy = int(y_mm / self.cell_size_mm)
        if 0 <= cx < self.cols and 0 <= cy < self.rows:
            return (cx, cy)
        return None

    def is_wall_cell(self, cx: int, cy: int) -> bool:
        pos = self.cell_to_pos((cx, cy))
        return not self.is_free_point(pos[0], pos[1], wall_clearance_mm=self.outer_wall_margin_mm)

    def is_free_point(self, x_mm: float, y_mm: float, wall_clearance_mm: Optional[float] = None) -> bool:
        clearance = self.outer_wall_margin_mm if wall_clearance_mm is None else wall_clearance_mm
        if x_mm < clearance or x_mm > self.width_mm - clearance:
            return False
        if y_mm < clearance or y_mm > self.height_mm - clearance:
            return False
        if y_mm < self.interior_wall_y_end_mm:
            divider_clearance = min(clearance, self.divider_margin_mm)
            if (self.interior_wall_x_left_mm - divider_clearance
                    < x_mm
                    < self.interior_wall_x_right_mm + divider_clearance):
                return False
        for tip_x in (self.interior_wall_x_left_mm, self.interior_wall_x_right_mm):
            if math.hypot(x_mm - tip_x, y_mm - self.interior_wall_y_end_mm) < self.divider_tip_clearance_mm:
                return False
        return True

    def explorable_cell_count(self) -> int:
        count = 0
        for cy in range(self.rows):
            for cx in range(self.cols):
                if not self.is_wall_cell(cx, cy):
                    count += 1
        return count

    def distance_to_config_space_obstacle(self, x_mm: float, y_mm: float) -> float:
        distances = [
            x_mm,
            self.width_mm - x_mm,
            y_mm,
            self.height_mm - y_mm,
        ]
        if y_mm < self.interior_wall_y_end_mm:
            if x_mm <= self.interior_wall_x_left_mm:
                distances.append(self.interior_wall_x_left_mm - x_mm)
            elif x_mm >= self.interior_wall_x_right_mm:
                distances.append(x_mm - self.interior_wall_x_right_mm)
            else:
                distances.append(0.0)
        for tip_x in (self.interior_wall_x_left_mm, self.interior_wall_x_right_mm):
            distances.append(math.hypot(x_mm - tip_x, y_mm - self.interior_wall_y_end_mm))
        return min(distances)

    def wall_segments(self) -> List[Tuple[float, float, float, float]]:
        return [
            (0.0, 0.0, self.width_mm, 0.0),
            (self.width_mm, 0.0, self.width_mm, self.height_mm),
            (self.width_mm, self.height_mm, 0.0, self.height_mm),
            (0.0, self.height_mm, 0.0, 0.0),
            (self.interior_wall_x_left_mm, 0.0, self.interior_wall_x_left_mm, self.interior_wall_y_end_mm),
            (self.interior_wall_x_right_mm, 0.0, self.interior_wall_x_right_mm, self.interior_wall_y_end_mm),
        ]

    def clamp_to_bounds(self, x_mm: float, y_mm: float, clearance_mm: Optional[float] = None) -> Tuple[float, float]:
        clearance = self.outer_wall_margin_mm if clearance_mm is None else clearance_mm
        return (
            max(clearance, min(self.width_mm - clearance, x_mm)),
            max(clearance, min(self.height_mm - clearance, y_mm)),
        )

    def divider_side(self, x_mm: float) -> str:
        if x_mm < self.interior_wall_x_left_mm:
            return "left"
        if x_mm > self.interior_wall_x_right_mm:
            return "right"
        return "divider"


@dataclass(frozen=True)
class ArucoCalibration:
    camera_width_px: int = 1280
    camera_height_px: int = 720
    cam_h_mm: float = 2190.0
    corner_h_mm: float = 152.0
    robot_tag_h_mm: float = 80.0
    camera_center_x_mm: float = 1000.0
    camera_center_y_mm: float = -40.0
    stable_corner_frames: int = 20
    position_rate_hz: float = 30.0

    @property
    def floor_scale(self) -> float:
        return self.cam_h_mm / (self.cam_h_mm - self.corner_h_mm)

    @property
    def robot_scale(self) -> float:
        return (self.cam_h_mm - self.robot_tag_h_mm) / (self.cam_h_mm - self.corner_h_mm)

    @property
    def corner_destination_points(self) -> List[List[float]]:
        return [
            [-115.0, 0.0],
            [2110.0, 0.0],
            [2110.0, 1500.0],
            [-113.0, 1500.0],
        ]


DEFAULT_ARENA = ArenaGeometry()
DEFAULT_ARUCO = ArucoCalibration()
HARDWARE_CLONE_STARTS: Dict[int, Tuple[float, float, float]] = {
    1: (185.9, 382.1, 0.188),
    2: (792.9, 709.2, 2.255),
    3: (318.8, 147.4, 0.736),
    4: (768.7, 219.2, 1.577),
    5: (182.4, 709.9, 0.410),
}


def hardware_overlay_header() -> Dict[str, object]:
    return {
        "arena_width_mm": DEFAULT_ARENA.width_mm,
        "arena_height_mm": DEFAULT_ARENA.height_mm,
        "cell_size": DEFAULT_ARENA.cell_size_mm,
        "interior_wall_x_left": DEFAULT_ARENA.interior_wall_x_left_mm,
        "interior_wall_x_right": DEFAULT_ARENA.interior_wall_x_right_mm,
        "interior_wall_y_end": DEFAULT_ARENA.interior_wall_y_end_mm,
        "robot_radius": DEFAULT_ARENA.robot_radius_mm,
        "body_half_width": DEFAULT_ARENA.body_half_width_mm,
        "swept_radius": DEFAULT_ARENA.swept_radius_mm,
        "cam_h": DEFAULT_ARUCO.cam_h_mm,
        "corner_h": DEFAULT_ARUCO.corner_h_mm,
        "robot_tag_h": DEFAULT_ARUCO.robot_tag_h_mm,
        "camera_center_x": DEFAULT_ARUCO.camera_center_x_mm,
        "camera_center_y": DEFAULT_ARUCO.camera_center_y_mm,
        "floor_scale": DEFAULT_ARUCO.floor_scale,
        "robot_scale": DEFAULT_ARUCO.robot_scale,
        "corner_dst_pts": DEFAULT_ARUCO.corner_destination_points,
    }
