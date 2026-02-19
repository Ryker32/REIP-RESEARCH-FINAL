#!/usr/bin/env python3
"""
REIP Robot - Local Belief Map
Converts ToF sensor readings -> Occupancy grid -> belief_lr for REIP.

This bridges your hardware to the REIP policy.
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# ============================================================================
# CONFIGURATION
# ============================================================================

# Map settings
MAP_RESOLUTION = 0.05       # 5cm per cell
MAP_SIZE_M = 5.0            # 5m x 5m local map
MAP_SIZE_CELLS = int(MAP_SIZE_M / MAP_RESOLUTION)  # 100x100

# ToF sensor angles (degrees from forward, counterclockwise positive)
TOF_ANGLES = {
    "front": 0,
    "front_left": 30,
    "front_right": -30,
    "left": 90,
    "right": -90,
}

# Sensor model
TOF_MAX_RANGE_M = 1.2       # VL53L0X max reliable range
TOF_MIN_RANGE_M = 0.03      # Minimum valid reading

# Occupancy update (log-odds)
L_OCC = 0.85                # Log-odds for occupied
L_FREE = -0.4               # Log-odds for free
L_PRIOR = 0.0               # Prior (unknown = 0.5 probability)
L_MIN = -2.0                # Clamp min
L_MAX = 3.5                 # Clamp max

# ============================================================================
# DATA STRUCTURES
# ============================================================================


@dataclass
class RobotPose:
    x: float = 0.0          # meters
    y: float = 0.0          # meters
    theta: float = 0.0      # radians (0 = +X, counterclockwise)


@dataclass
class ToFReading:
    front: int = 0          # mm
    front_left: int = 0
    front_right: int = 0
    left: int = 0
    right: int = 0

# ============================================================================
# OCCUPANCY GRID
# ============================================================================


class OccupancyGrid:
    """
    Local occupancy grid using log-odds representation.

    Integrates ToF readings via ray casting.
    Outputs belief_lr compatible with REIP (-1 unknown, 0 free, 2 obstacle).
    """

    def __init__(self, resolution=MAP_RESOLUTION, size_cells=MAP_SIZE_CELLS):
        self.resolution = resolution
        self.size = size_cells

        # Log-odds grid (0 = unknown = P=0.5)
        self.grid = np.full((size_cells, size_cells), L_PRIOR, dtype=np.float32)

        # Tracking grids for REIP
        self.seen_world = np.zeros((size_cells, size_cells), dtype=bool)
        self.obstacle_world = np.zeros((size_cells, size_cells), dtype=bool)

        # Robot starts at center of map
        self.origin_x = size_cells // 2
        self.origin_y = size_cells // 2

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (m) to grid indices"""
        gx = int(x / self.resolution) + self.origin_x
        gy = int(y / self.resolution) + self.origin_y
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (m)"""
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return x, y

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size and 0 <= gy < self.size

    def ray_cast(self, x0: float, y0: float, angle: float, distance: float) -> List[Tuple[int, int]]:
        """
        Cast ray from (x0, y0) at angle for distance.
        Returns list of grid cells along ray.
        Uses Bresenham's algorithm.
        """
        # End point
        x1 = x0 + distance * math.cos(angle)
        y1 = y0 + distance * math.sin(angle)

        # Convert to grid
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)

        # Bresenham's line algorithm
        cells = []
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy

        gx, gy = gx0, gy0
        while True:
            if self.in_bounds(gx, gy):
                cells.append((gx, gy))

            if gx == gx1 and gy == gy1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx += sx
            if e2 < dx:
                err += dx
                gy += sy

        return cells

    def update_from_tof(self, pose: RobotPose, readings: ToFReading):
        """
        Update occupancy grid from ToF readings.

        For each sensor:
        1. Ray cast from robot pose
        2. Mark cells along ray as FREE
        3. Mark hit cell as OCCUPIED (if valid reading)
        """
        for sensor_name, angle_deg in TOF_ANGLES.items():
            # Get reading in meters
            reading_mm = getattr(readings, sensor_name, 0)
            if reading_mm <= 0:
                continue

            reading_m = reading_mm / 1000.0

            # Skip invalid readings
            if reading_m < TOF_MIN_RANGE_M:
                continue

            # Sensor angle in world frame
            sensor_angle = pose.theta + math.radians(angle_deg)

            # Determine if we hit something or max range
            hit_obstacle = reading_m < TOF_MAX_RANGE_M

            # Ray cast
            ray_dist = min(reading_m, TOF_MAX_RANGE_M)
            cells = self.ray_cast(pose.x, pose.y, sensor_angle, ray_dist)

            if not cells:
                continue

            # Update cells along ray as FREE
            for cell in cells[:-1]:  # All but last
                gx, gy = cell
                self.grid[gx, gy] = np.clip(self.grid[gx, gy] + L_FREE, L_MIN, L_MAX)
                self.seen_world[gx, gy] = True

            # Update last cell
            gx, gy = cells[-1]
            if hit_obstacle:
                # Mark as OCCUPIED
                self.grid[gx, gy] = np.clip(self.grid[gx, gy] + L_OCC, L_MIN, L_MAX)
                self.obstacle_world[gx, gy] = True
            else:
                # Max range = FREE
                self.grid[gx, gy] = np.clip(self.grid[gx, gy] + L_FREE, L_MIN, L_MAX)
            self.seen_world[gx, gy] = True

    def get_probability(self, gx: int, gy: int) -> float:
        """Convert log-odds to probability"""
        if not self.in_bounds(gx, gy):
            return 0.5
        l = self.grid[gx, gy]
        return 1.0 - 1.0 / (1.0 + math.exp(l))

    def get_belief_lr(self, downsample: int = 2) -> np.ndarray:
        """
        Get coarse belief grid for REIP.

        Returns int8 grid with:
          -1 = unknown, 0 = free, 2 = obstacle
        """
        ds = max(1, int(downsample))
        lr_size = self.size // ds
        belief_lr = np.full((lr_size, lr_size), -1, dtype=np.int8)

        for i in range(lr_size):
            for j in range(lr_size):
                x0, x1 = i * ds, min(self.size, (i + 1) * ds)
                y0, y1 = j * ds, min(self.size, (j + 1) * ds)

                seen_block = self.seen_world[x0:x1, y0:y1]
                obstacle_block = self.obstacle_world[x0:x1, y0:y1]

                if obstacle_block.any():
                    belief_lr[i, j] = 2
                elif seen_block.any():
                    belief_lr[i, j] = 0
                else:
                    belief_lr[i, j] = -1

        return belief_lr

# ============================================================================
# FRONTIER DETECTION
# ============================================================================


class FrontierDetector:
    """
    Detect frontiers (boundaries between known and unknown space).

    Frontiers are exploration targets for REIP.
    """

    def __init__(self, grid: OccupancyGrid):
        self.grid = grid

    def detect(self, min_size: int = 3) -> List[Tuple[int, int]]:
        """
        Find frontier cells.

        A frontier cell is:
        - SEEN (explored)
        - FREE (not obstacle)
        - Adjacent to UNSEEN cell
        """
        frontiers = []

        for gx in range(1, self.grid.size - 1):
            for gy in range(1, self.grid.size - 1):
                # Must be seen and free
                if not self.grid.seen_world[gx, gy]:
                    continue
                if self.grid.obstacle_world[gx, gy]:
                    continue

                # Check for adjacent unseen
                has_unseen_neighbor = False
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = gx + dx, gy + dy
                    if not self.grid.seen_world[nx, ny]:
                        has_unseen_neighbor = True
                        break

                if has_unseen_neighbor:
                    frontiers.append((gx, gy))

        # Cluster and filter small frontiers
        if min_size > 1:
            frontiers = self._filter_small_frontiers(frontiers, min_size)

        return frontiers

    def _filter_small_frontiers(
        self,
        frontiers: List[Tuple[int, int]],
        min_size: int,
    ) -> List[Tuple[int, int]]:
        """Remove isolated frontier cells"""
        if not frontiers:
            return []

        frontier_set = set(frontiers)
        filtered = []

        for f in frontiers:
            # Count neighbors that are also frontiers
            neighbors = 0
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    if (f[0] + dx, f[1] + dy) in frontier_set:
                        neighbors += 1

            if neighbors >= min_size - 1:
                filtered.append(f)

        return filtered

    def get_frontier_centroids(self) -> List[Tuple[float, float]]:
        """
        Get frontier cluster centroids in world coordinates.
        These are the exploration targets for REIP.
        """
        frontiers = self.detect()
        if not frontiers:
            return []

        # Simple clustering by proximity
        clusters = []
        used = set()

        for f in frontiers:
            if f in used:
                continue

            # BFS to find connected frontier cells
            cluster = [f]
            queue = [f]
            used.add(f)

            while queue:
                current = queue.pop(0)
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        neighbor = (current[0] + dx, current[1] + dy)
                        if neighbor in frontiers and neighbor not in used:
                            used.add(neighbor)
                            cluster.append(neighbor)
                            queue.append(neighbor)

            clusters.append(cluster)

        # Compute centroids
        centroids = []
        for cluster in clusters:
            avg_gx = sum(c[0] for c in cluster) / len(cluster)
            avg_gy = sum(c[1] for c in cluster) / len(cluster)
            wx, wy = self.grid.grid_to_world(int(avg_gx), int(avg_gy))
            centroids.append((wx, wy))

        return centroids

# ============================================================================
# REIP BELIEF INTERFACE
# ============================================================================


class REIPBeliefInterface:
    """
    Interface between hardware and REIP policy.

    Maintains local belief map, detects frontiers,
    computes predicted/observed gain for trust updates.
    """

    def __init__(self):
        self.grid = OccupancyGrid()
        self.frontier_detector = FrontierDetector(self.grid)
        self.pose = RobotPose()

        # For gain calculation
        self._last_unknown_count = 0

    def update_pose(self, x: float, y: float, theta: float):
        """Update robot pose from odometry"""
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta

    def update_sensors(self, readings: ToFReading):
        """Update belief map from ToF readings"""
        self.grid.update_from_tof(self.pose, readings)

    def get_frontiers(self) -> List[Tuple[float, float]]:
        """Get frontier targets for REIP assignment"""
        return self.frontier_detector.get_frontier_centroids()

    def get_belief_lr(self) -> np.ndarray:
        """Get coarse belief grid for REIP"""
        return self.grid.get_belief_lr()

    def predict_gain(self, target: Tuple[float, float], radius: float = 0.5) -> float:
        """
        Predict information gain for moving to target.

        Counts unknown cells within radius of target.
        This is the leader's "promise" in REIP.
        """
        gx, gy = self.grid.world_to_grid(target[0], target[1])
        r_cells = int(radius / self.grid.resolution)

        unknown_count = 0
        total_count = 0

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                nx, ny = gx + dx, gy + dy
                if not self.grid.in_bounds(nx, ny):
                    continue
                total_count += 1
                if not self.grid.seen_world[nx, ny]:
                    unknown_count += 1

        self._last_unknown_count = unknown_count

        # Normalize to 0-1
        if total_count == 0:
            return 0.0
        return unknown_count / total_count

    def observe_gain(self, target: Tuple[float, float], radius: float = 0.5) -> float:
        """
        Observe actual information gain after moving.

        Compares current unknown count to predicted.
        This validates the leader's promise.
        """
        gx, gy = self.grid.world_to_grid(target[0], target[1])
        r_cells = int(radius / self.grid.resolution)

        remaining_unknown = 0
        total_count = 0

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                nx, ny = gx + dx, gy + dy
                if not self.grid.in_bounds(nx, ny):
                    continue
                total_count += 1
                if not self.grid.seen_world[nx, ny]:
                    remaining_unknown += 1

        # Gain = reduction in unknown cells
        if self._last_unknown_count == 0:
            return 0.0

        revealed = self._last_unknown_count - remaining_unknown
        observed_gain = max(0.0, revealed / self._last_unknown_count)

        return observed_gain

    def get_seen_world(self) -> np.ndarray:
        """For REIP compatibility"""
        return self.grid.seen_world.copy()

    def get_obstacle_world(self) -> np.ndarray:
        """For REIP compatibility"""
        return self.grid.obstacle_world.copy()

# ============================================================================
# TEST
# ============================================================================


if __name__ == "__main__":
    print("REIP Belief Map Test")
    print("=" * 40)

    # Create interface
    reip = REIPBeliefInterface()

    # Simulate robot at origin, facing +X
    reip.update_pose(0, 0, 0)

    # Simulate ToF readings (mm)
    readings = ToFReading(
        front=500,       # 50cm ahead
        front_left=800,  # 80cm front-left
        front_right=300, # 30cm front-right (close obstacle)
        left=1200,       # 1.2m left (max range)
        right=600,       # 60cm right
    )

    # Update belief
    reip.update_sensors(readings)

    # Get frontiers
    frontiers = reip.get_frontiers()
    print(f"\nFrontiers detected: {len(frontiers)}")
    for i, f in enumerate(frontiers[:5]):
        print(f"  {i + 1}. ({f[0]:.2f}, {f[1]:.2f}) m")

    # Get belief for REIP
    belief_lr = reip.get_belief_lr()
    print(f"\nBelief grid shape: {belief_lr.shape}")
    print(f"Unknown cells: {int((belief_lr == -1).sum())}")

    # Test gain prediction
    if frontiers:
        target = frontiers[0]
        pred = reip.predict_gain(target)
        print(f"\nPredicted gain for {target}: {pred:.3f}")

    print("\nDone!")
