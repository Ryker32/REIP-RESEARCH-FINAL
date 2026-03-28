#!/usr/bin/env python3
"""
Simple REIP Visual Simulation - Everything in ONE process.
No networking, no subprocesses. Just like the original simulation.
"""

import pygame
import math
import random
import time
from dataclasses import dataclass, field
from typing import Dict, Set, List, Tuple, Optional
from enum import Enum

# ============== Config ==============
ARENA_WIDTH = 2000   # mm
ARENA_HEIGHT = 1500  # mm
CELL_SIZE = 125      # mm
NUM_ROBOTS = 5
SIM_RATE = 30        # FPS
ROBOT_SPEED = 8      # pixels per frame

# Trust parameters
TRUST_THRESHOLD = 0.5
IMPEACHMENT_THRESHOLD = 0.3
MIN_TRUST = 0.1
SUSPICION_THRESHOLD = 1.5
WEIGHT_VISITED = 1.0
WEIGHT_PEER = 0.3

# Display
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800
MARGIN = 50

# Colors
COLOR_BG = (30, 30, 35)
COLOR_GRID = (50, 50, 55)
COLOR_VISITED = (60, 90, 60)
COLOR_ROBOT = (100, 180, 255)
COLOR_LEADER = (255, 220, 100)
COLOR_BAD = (255, 100, 100)
COLOR_TEXT = (220, 220, 220)

# ============== Robot Class ==============
@dataclass
class Robot:
    robot_id: int
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    # Coverage
    my_visited: Set[tuple] = field(default_factory=set)
    
    # Trust
    trust_in_leader: float = 1.0
    suspicion: float = 0.0
    
    # State
    is_leader: bool = False
    current_leader: int = 1
    target: Optional[Tuple[float, float]] = None
    
    # Faults
    bad_leader_mode: bool = False
    motor_fault: Optional[str] = None

class REIPSimulation:
    def __init__(self, num_robots: int = NUM_ROBOTS):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("REIP Simple Simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('consolas', 14)
        self.font_large = pygame.font.SysFont('consolas', 18)
        
        self.num_robots = num_robots
        self.scale = min(
            (WINDOW_WIDTH - 2 * MARGIN) / ARENA_WIDTH,
            (WINDOW_HEIGHT - 2 * MARGIN - 100) / ARENA_HEIGHT
        )
        
        # Create robots
        self.robots: Dict[int, Robot] = {}
        for i in range(1, num_robots + 1):
            self.robots[i] = Robot(
                robot_id=i,
                x=random.uniform(200, ARENA_WIDTH - 200),
                y=random.uniform(200, ARENA_HEIGHT - 200),
                theta=random.uniform(0, 2 * math.pi)
            )
        
        # First robot is leader
        self.robots[1].is_leader = True
        self.current_leader = 1
        
        # Global state
        self.known_visited: Set[tuple] = set()
        self.leader_failures: Dict[int, int] = {}
        self.running = True
        self.tick = 0
    
    def get_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        cx = int(x / CELL_SIZE)
        cy = int(y / CELL_SIZE)
        if 0 <= cx < ARENA_WIDTH // CELL_SIZE and 0 <= cy < ARENA_HEIGHT // CELL_SIZE:
            return (cx, cy)
        return None
    
    def cell_to_pos(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        return ((cell[0] + 0.5) * CELL_SIZE, (cell[1] + 0.5) * CELL_SIZE)
    
    def distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def get_frontiers(self) -> List[Tuple[int, int]]:
        """Get unexplored cells"""
        frontiers = []
        for cx in range(ARENA_WIDTH // CELL_SIZE):
            for cy in range(ARENA_HEIGHT // CELL_SIZE):
                if (cx, cy) not in self.known_visited:
                    frontiers.append((cx, cy))
        return frontiers
    
    def compute_assignments(self) -> Dict[int, Tuple[float, float]]:
        """Leader computes frontier assignments for all robots"""
        leader = self.robots[self.current_leader]
        
        # Bad leader sends robots to explored cells
        if leader.bad_leader_mode and self.known_visited:
            explored = list(self.known_visited)
            assignments = {}
            for rid in self.robots:
                bad_cell = random.choice(explored)
                assignments[rid] = self.cell_to_pos(bad_cell)
            return assignments
        
        # Normal: assign nearest unexplored frontier to each robot
        frontiers = self.get_frontiers()
        if not frontiers:
            return {}
        
        assignments = {}
        assigned = set()
        
        # Sort robots by distance to nearest frontier
        def nearest_dist(rid):
            pos = (self.robots[rid].x, self.robots[rid].y)
            return min(self.distance(pos, self.cell_to_pos(f)) for f in frontiers)
        
        for rid in sorted(self.robots.keys(), key=nearest_dist):
            robot = self.robots[rid]
            pos = (robot.x, robot.y)
            
            best = None
            best_dist = float('inf')
            for f in frontiers:
                if f in assigned:
                    continue
                fp = self.cell_to_pos(f)
                d = self.distance(pos, fp)
                if d < best_dist:
                    best_dist = d
                    best = f
            
            if best:
                assigned.add(best)
                assignments[rid] = self.cell_to_pos(best)
        
        return assignments
    
    def assess_command(self, robot: Robot, target: Tuple[float, float]):
        """Follower assesses if leader's command is good"""
        if robot.is_leader:
            return
        
        cell = self.get_cell(target[0], target[1])
        if not cell:
            return
        
        bad = False
        weight = 0.0
        
        # Check if target is already explored (by me)
        if cell in robot.my_visited:
            bad = True
            weight = WEIGHT_VISITED
        # Check if target is in known_visited (from peers)
        elif cell in self.known_visited:
            bad = True
            weight = WEIGHT_PEER
        
        # MPC: Check if target direction is wildly different from ANY frontier
        # (Improved: checks against NEAREST frontier, not centroid)
        frontiers = self.get_frontiers()
        if frontiers:
            cmd_dir = math.atan2(target[1] - robot.y, target[0] - robot.x)
            
            # Find best alignment with ANY frontier
            best_alignment = math.pi  # Start with worst (180 deg)
            for f in frontiers:
                fp = self.cell_to_pos(f)
                frontier_dir = math.atan2(fp[1] - robot.y, fp[0] - robot.x)
                angle_diff = abs(frontier_dir - cmd_dir)
                if angle_diff > math.pi:
                    angle_diff = 2 * math.pi - angle_diff
                if angle_diff < best_alignment:
                    best_alignment = angle_diff
                # Early exit if good alignment found
                if best_alignment < math.pi / 4:
                    break
            
            # Only penalize if command isn't close to ANY frontier (severe mismatch)
            SEVERE_THRESHOLD = math.pi * 3 / 4  # 135 deg
            if best_alignment > SEVERE_THRESHOLD:
                mpc_weight = 0.5 + (best_alignment - SEVERE_THRESHOLD) / (math.pi - SEVERE_THRESHOLD) * 0.3
                weight += mpc_weight
                bad = True
                print(f"[MPC SEVERE] R{robot.robot_id}: No frontier within 135 deg of cmd!")
        
        if bad:
            robot.suspicion += weight
        else:
            robot.suspicion = max(0, robot.suspicion - 0.1)
        
        # Trust decay
        if robot.suspicion >= SUSPICION_THRESHOLD:
            robot.trust_in_leader = max(MIN_TRUST, robot.trust_in_leader - 0.1)
            robot.suspicion -= SUSPICION_THRESHOLD
    
    def run_election(self):
        """Run leader election"""
        # Collect trust scores
        avg_trust = sum(r.trust_in_leader for r in self.robots.values() if not r.is_leader) / max(1, self.num_robots - 1)
        
        # Check impeachment
        if avg_trust < IMPEACHMENT_THRESHOLD:
            old_leader = self.current_leader
            self.leader_failures[old_leader] = self.leader_failures.get(old_leader, 0) + 1
            self.robots[old_leader].is_leader = False
            self.robots[old_leader].bad_leader_mode = False
            
            # Elect new leader: highest trust, fewest failures, lowest ID
            candidates = []
            for rid, robot in self.robots.items():
                if rid != old_leader:  # Don't re-elect impeached leader
                    score = (1.0, -self.leader_failures.get(rid, 0), -rid)
                    candidates.append((score, rid))
            
            candidates.sort(reverse=True)
            new_leader = candidates[0][1]
            
            self.current_leader = new_leader
            self.robots[new_leader].is_leader = True
            
            # Reset trust for new leader
            for robot in self.robots.values():
                robot.trust_in_leader = 1.0
                robot.suspicion = 0.0
                robot.current_leader = new_leader
            
            print(f"[ELECTION] Leader {old_leader} -> {new_leader} (failures: {self.leader_failures})")
    
    def update(self):
        """Main update loop"""
        self.tick += 1
        
        # Update visited cells
        for robot in self.robots.values():
            cell = self.get_cell(robot.x, robot.y)
            if cell:
                robot.my_visited.add(cell)
                self.known_visited.add(cell)
        
        # Leader computes assignments
        assignments = self.compute_assignments()
        
        # Followers assess and move
        for rid, robot in self.robots.items():
            # Handle motor faults
            if robot.motor_fault == 'spin':
                robot.theta += 0.2
                continue
            elif robot.motor_fault == 'stop':
                continue
            
            # Get assignment
            target = assignments.get(rid)
            if target:
                robot.target = target
                
                # Followers assess command
                if not robot.is_leader:
                    self.assess_command(robot, target)
            
            # Move toward target
            if robot.target:
                dx = robot.target[0] - robot.x
                dy = robot.target[1] - robot.y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist > 20:
                    robot.theta = math.atan2(dy, dx)
                    robot.x += ROBOT_SPEED * math.cos(robot.theta)
                    robot.y += ROBOT_SPEED * math.sin(robot.theta)
                else:
                    robot.target = None
            
            # Clamp to arena
            robot.x = max(50, min(ARENA_WIDTH - 50, robot.x))
            robot.y = max(50, min(ARENA_HEIGHT - 50, robot.y))
            
            # Update leader tracking
            robot.current_leader = self.current_leader
        
        # Run election periodically
        if self.tick % 60 == 0:  # Every 2 seconds at 30 FPS
            self.run_election()
    
    def draw(self):
        self.screen.fill(COLOR_BG)
        
        # Grid
        cells_x = ARENA_WIDTH // CELL_SIZE
        cells_y = ARENA_HEIGHT // CELL_SIZE
        
        for cx in range(cells_x):
            for cy in range(cells_y):
                x1 = int(MARGIN + cx * CELL_SIZE * self.scale)
                y1 = int(MARGIN + (ARENA_HEIGHT - (cy + 1) * CELL_SIZE) * self.scale)
                w = int(CELL_SIZE * self.scale)
                h = int(CELL_SIZE * self.scale)
                
                rect = pygame.Rect(x1, y1, w, h)
                
                if (cx, cy) in self.known_visited:
                    pygame.draw.rect(self.screen, COLOR_VISITED, rect)
                
                pygame.draw.rect(self.screen, COLOR_GRID, rect, 1)
        
        # Robots
        for rid, robot in self.robots.items():
            sx = int(MARGIN + robot.x * self.scale)
            sy = int(MARGIN + (ARENA_HEIGHT - robot.y) * self.scale)
            
            # Color based on state
            if robot.bad_leader_mode:
                color = COLOR_BAD
            elif robot.is_leader:
                color = COLOR_LEADER
            elif robot.trust_in_leader < TRUST_THRESHOLD:
                color = (255, 150, 100)  # Low trust
            else:
                color = COLOR_ROBOT
            
            # Draw robot
            pygame.draw.circle(self.screen, color, (sx, sy), 15)
            pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), 15, 2)
            
            # Direction line
            ex = sx + int(20 * math.cos(robot.theta))
            ey = sy - int(20 * math.sin(robot.theta))
            pygame.draw.line(self.screen, (255, 255, 255), (sx, sy), (ex, ey), 2)
            
            # ID
            text = self.font.render(str(rid), True, (0, 0, 0))
            self.screen.blit(text, (sx - 5, sy - 6))
        
        # Stats
        y = WINDOW_HEIGHT - 80
        
        # Leader
        leader_text = self.font_large.render(f"Leader: Robot {self.current_leader}", True, COLOR_LEADER)
        self.screen.blit(leader_text, (20, y))
        
        # Coverage
        total_cells = cells_x * cells_y
        cov_pct = len(self.known_visited) / total_cells * 100
        cov_text = self.font_large.render(f"Coverage: {len(self.known_visited)}/{total_cells} ({cov_pct:.1f}%)", True, COLOR_TEXT)
        self.screen.blit(cov_text, (250, y))
        
        y += 25
        
        # Trust values
        x = 20
        for rid, robot in self.robots.items():
            trust_color = COLOR_TEXT if robot.trust_in_leader > TRUST_THRESHOLD else COLOR_BAD
            status = f"R{rid}: T={robot.trust_in_leader:.2f}"
            if robot.is_leader:
                status += " (L)"
            if robot.bad_leader_mode:
                status += " [BAD]"
            text = self.font.render(status, True, trust_color)
            self.screen.blit(text, (x, y))
            x += 150
        
        y += 20
        ctrl_text = self.font.render("Keys: 1-5=bad_leader  SHIFT+1-5=spin  C=clear  Q=quit", True, (150, 150, 150))
        self.screen.blit(ctrl_text, (20, y))
        
        pygame.display.flip()
    
    def handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            if event.type == pygame.KEYDOWN:
                mods = pygame.key.get_mods()
                shift = mods & pygame.KMOD_SHIFT
                
                if event.key == pygame.K_q:
                    return False
                elif event.key == pygame.K_c:
                    for robot in self.robots.values():
                        robot.bad_leader_mode = False
                        robot.motor_fault = None
                    print("All faults cleared")
                elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5):
                    rid = event.key - pygame.K_0
                    if rid <= self.num_robots:
                        if shift:
                            self.robots[rid].motor_fault = 'spin'
                            print(f"Robot {rid}: SPIN fault")
                        else:
                            self.robots[rid].bad_leader_mode = True
                            print(f"Robot {rid}: BAD_LEADER mode")
        
        return True
    
    def run(self):
        print("\nSimple REIP Simulation")
        print("=" * 40)
        print("Keys: 1-5 = bad_leader fault")
        print("      SHIFT+1-5 = spin fault")
        print("      C = clear faults, Q = quit\n")
        
        while self.running:
            if not self.handle_events():
                break
            
            self.update()
            self.draw()
            self.clock.tick(SIM_RATE)
        
        pygame.quit()

if __name__ == "__main__":
    sim = REIPSimulation()
    sim.run()
