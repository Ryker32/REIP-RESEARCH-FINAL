#!/usr/bin/env python3
"""
REIP Swarm Visualizer
Displays real-time swarm state by listening to robot broadcasts.
For monitoring only - does not control anything.

Usage: python3 visualizer.py
"""

import socket
import json
import time
import threading
import math
from dataclasses import dataclass
from typing import Dict, Optional

# Try pygame for visualization
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("pygame not installed - using text mode")
    print("Install with: pip install pygame")

# ============== Configuration ==============
UDP_PEER_PORT = 5004
UDP_POSITION_PORT = 5003

# Display
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 500
ARENA_WIDTH = 2000   # mm
ARENA_HEIGHT = 1000  # mm
CELL_SIZE = 150      # mm - robot-sized

# Colors
COLOR_BG = (30, 30, 40)
COLOR_GRID = (50, 50, 60)
COLOR_VISITED = (60, 100, 60)
COLOR_ROBOT = (100, 200, 100)
COLOR_LEADER = (255, 215, 0)
COLOR_LOW_TRUST = (200, 80, 80)
COLOR_TEXT = (200, 200, 200)

# ============== Data ==============
@dataclass
class RobotVis:
    robot_id: int
    x: float = 0
    y: float = 0
    theta: float = 0
    state: str = "idle"
    trust: float = 1.0
    vote: Optional[int] = None
    coverage: int = 0
    last_seen: float = 0

# ============== Visualizer ==============
class Visualizer:
    def __init__(self):
        self.robots: Dict[int, RobotVis] = {}
        self.visited_cells: set = set()
        self.running = False
        
        # Network
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', UDP_PEER_PORT))
        self.socket.setblocking(False)
        
        # Pygame
        if PYGAME_AVAILABLE:
            pygame.init()
            self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
            pygame.display.set_caption("REIP Swarm Visualizer")
            self.font = pygame.font.Font(None, 24)
            self.small_font = pygame.font.Font(None, 18)
        
        # Scale factors
        self.scale_x = (WINDOW_WIDTH - 200) / ARENA_WIDTH  # Leave space for stats
        self.scale_y = WINDOW_HEIGHT / ARENA_HEIGHT
        self.offset_x = 0
        self.offset_y = 0
    
    def arena_to_screen(self, x: float, y: float) -> tuple:
        """Convert arena coords (mm) to screen coords (pixels)"""
        sx = int(x * self.scale_x + self.offset_x)
        sy = int(WINDOW_HEIGHT - y * self.scale_y + self.offset_y)
        return (sx, sy)
    
    def receive_data(self):
        """Receive robot broadcasts"""
        try:
            while True:
                data, _ = self.socket.recvfrom(4096)
                msg = json.loads(data.decode())
                
                if msg.get('type') == 'peer_state':
                    rid = msg.get('robot_id')
                    if rid:
                        if rid not in self.robots:
                            self.robots[rid] = RobotVis(robot_id=rid)
                        
                        r = self.robots[rid]
                        r.x = msg.get('x', 0)
                        r.y = msg.get('y', 0)
                        r.theta = msg.get('theta', 0)
                        r.state = msg.get('state', 'idle')
                        r.trust = msg.get('trust', 1.0)
                        r.vote = msg.get('vote')
                        r.coverage = msg.get('coverage_count', 0)
                        r.last_seen = time.time()
                        
                        # Track visited cells
                        for cell in msg.get('visited_cells', []):
                            self.visited_cells.add(tuple(cell))
        except BlockingIOError:
            pass
    
    def draw(self):
        """Draw the visualization"""
        if not PYGAME_AVAILABLE:
            return
        
        self.screen.fill(COLOR_BG)
        
        # Draw grid
        for x in range(0, ARENA_WIDTH + CELL_SIZE, CELL_SIZE):
            sx, _ = self.arena_to_screen(x, 0)
            pygame.draw.line(self.screen, COLOR_GRID, (sx, 0), (sx, WINDOW_HEIGHT), 1)
        for y in range(0, ARENA_HEIGHT + CELL_SIZE, CELL_SIZE):
            _, sy = self.arena_to_screen(0, y)
            pygame.draw.line(self.screen, COLOR_GRID, (0, sy), (WINDOW_WIDTH - 200, sy), 1)
        
        # Draw visited cells
        for cell in self.visited_cells:
            cx, cy = cell
            x = cx * CELL_SIZE
            y = cy * CELL_SIZE
            sx, sy = self.arena_to_screen(x, y + CELL_SIZE)
            w = int(CELL_SIZE * self.scale_x)
            h = int(CELL_SIZE * self.scale_y)
            pygame.draw.rect(self.screen, COLOR_VISITED, (sx, sy, w, h))
        
        # Find leader
        votes = {}
        for r in self.robots.values():
            if r.vote:
                votes[r.vote] = votes.get(r.vote, 0) + 1
        leader = max(votes.items(), key=lambda x: x[1])[0] if votes else None
        
        # Draw robots
        for rid, r in self.robots.items():
            if time.time() - r.last_seen > 3.0:
                continue  # Skip stale robots
            
            sx, sy = self.arena_to_screen(r.x, r.y)
            
            # Color based on state
            if rid == leader:
                color = COLOR_LEADER
            elif r.trust < 0.5:
                color = COLOR_LOW_TRUST
            else:
                color = COLOR_ROBOT
            
            # Body
            pygame.draw.circle(self.screen, color, (sx, sy), 15)
            
            # Heading arrow
            ax = sx + int(25 * math.cos(r.theta))
            ay = sy - int(25 * math.sin(r.theta))
            pygame.draw.line(self.screen, color, (sx, sy), (ax, ay), 3)
            
            # ID label
            label = self.small_font.render(f"R{rid}", True, COLOR_TEXT)
            self.screen.blit(label, (sx - 10, sy - 30))
        
        # Stats panel
        panel_x = WINDOW_WIDTH - 190
        pygame.draw.rect(self.screen, (40, 40, 50), (panel_x, 0, 200, WINDOW_HEIGHT))
        
        # Title
        title = self.font.render("REIP Swarm", True, COLOR_TEXT)
        self.screen.blit(title, (panel_x + 10, 10))
        
        # Coverage
        total_cells = (ARENA_WIDTH // CELL_SIZE) * (ARENA_HEIGHT // CELL_SIZE)
        coverage = len(self.visited_cells) / total_cells * 100 if total_cells > 0 else 0
        cov_text = self.font.render(f"Coverage: {coverage:.1f}%", True, COLOR_TEXT)
        self.screen.blit(cov_text, (panel_x + 10, 40))
        
        # Leader
        leader_text = self.font.render(f"Leader: R{leader}" if leader else "Leader: None", True, COLOR_LEADER)
        self.screen.blit(leader_text, (panel_x + 10, 70))
        
        # Robot list
        y = 110
        for rid in sorted(self.robots.keys()):
            r = self.robots[rid]
            if time.time() - r.last_seen > 3.0:
                continue
            
            # Trust bar
            bar_w = int(r.trust * 100)
            bar_color = COLOR_ROBOT if r.trust > 0.5 else COLOR_LOW_TRUST
            pygame.draw.rect(self.screen, (60, 60, 70), (panel_x + 10, y, 100, 10))
            pygame.draw.rect(self.screen, bar_color, (panel_x + 10, y, bar_w, 10))
            
            # Text
            info = self.small_font.render(f"R{rid}: {r.state[:6]} T={r.trust:.2f}", True, COLOR_TEXT)
            self.screen.blit(info, (panel_x + 10, y + 12))
            
            y += 40
        
        pygame.display.flip()
    
    def run_text_mode(self):
        """Fallback text-only mode"""
        print("=== REIP Swarm Monitor (text mode) ===\n")
        last_print = 0
        
        while self.running:
            self.receive_data()
            
            if time.time() - last_print > 1.0:
                total_cells = (ARENA_WIDTH // CELL_SIZE) * (ARENA_HEIGHT // CELL_SIZE)
                coverage = len(self.visited_cells) / total_cells * 100 if total_cells > 0 else 0
                
                print(f"\n--- Coverage: {coverage:.1f}% ---")
                for rid in sorted(self.robots.keys()):
                    r = self.robots[rid]
                    if time.time() - r.last_seen < 3.0:
                        print(f"  R{rid}: ({r.x:.0f},{r.y:.0f}) state={r.state} trust={r.trust:.2f} vote=R{r.vote}")
                
                last_print = time.time()
            
            time.sleep(0.1)
    
    def run(self):
        """Main loop"""
        self.running = True
        
        if not PYGAME_AVAILABLE:
            try:
                self.run_text_mode()
            except KeyboardInterrupt:
                pass
            return
        
        clock = pygame.time.Clock()
        
        print("Visualizer running... Close window to stop\n")
        
        while self.running:
            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            # Receive data
            self.receive_data()
            
            # Draw
            self.draw()
            
            clock.tick(30)
        
        pygame.quit()
        print("Stopped.")

# ============== Entry ==============
if __name__ == "__main__":
    vis = Visualizer()
    vis.run()
