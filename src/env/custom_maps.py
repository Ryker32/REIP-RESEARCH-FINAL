"""
Custom Map Loader for GridWorld Environments

Allows creating specific office layouts, mazes, or custom environments
instead of random obstacle generation.

Map Format:
- Text file with ASCII art
- '.' or ' ' = free space (walkable)
- '#' or 'X' = obstacle (wall, furniture)
- 'S' = starting area (suggested spawn points)

Example office map:
    ####################
    #........#.........#
    #...##...#....##...#
    #...##...S....##...#
    #........#.........#
    ####################
"""

import numpy as np
import random
from typing import List, Tuple


def load_office_layout(map_name: str = "default") -> np.ndarray:
    """
    Load a predefined office layout.
    
    Args:
        map_name: Name of the office layout to load
        
    Returns:
        grid: 2D numpy array where 0=free, -1=obstacle
    """
    layouts = {
        "default": _create_default_office(),
        "office": _create_office_layout(),
        "cubicles": _create_cubicle_layout(),
        "warehouse": _create_warehouse_layout(),
        "maze": _create_maze_layout(),
        "open_plan": _create_open_plan_office(),
    }
    
    if map_name not in layouts:
        print(f"Unknown map '{map_name}', using 'default'")
        map_name = "default"
    
    return layouts[map_name]


def load_map_from_file(filepath: str, size: int = 50) -> np.ndarray:
    """
    Load map from a text file - SIMPLE VERSION.
    
    Just place obstacles wherever you want!
    
    File format:
        # = obstacle/wall
        (anything else) = free space
    
    That's it! No special characters needed.
    
    Example file (my_office.txt):
        ####################
        #                  #
        #  ###      ###    #
        #  ###      ###    #
        #                  #
        ####################
    
    Args:
        filepath: Path to text file
        size: Desired grid size (will pad/crop to fit)
        
    Returns:
        grid: 2D numpy array where 0=free, -1=obstacle
    """
    try:
        with open(filepath, 'r') as f:
            lines = [line.rstrip('\n\r') for line in f.readlines()]
        
        # Remove empty lines at start/end
        while lines and not lines[0].strip():
            lines.pop(0)
        while lines and not lines[-1].strip():
            lines.pop()
        
        if not lines:
            print(f"Empty map file: {filepath}")
            print("Creating default layout instead")
            return _create_default_office(size)
        
        # Create grid (start with all free space)
        grid = np.zeros((size, size), dtype=np.int8)
        
        # Fill in obstacles from file
        for y, line in enumerate(lines):
            if y >= size:
                break
            for x, char in enumerate(line):
                if x >= size:
                    break
                if char == '#':  # Only # is obstacle, everything else is free
                    grid[y, x] = -1
        
        print(f"Loaded map from {filepath}: {len(lines)} rows")
        return grid
        
    except FileNotFoundError:
        print(f"File not found: {filepath}")
        print("Creating default office layout instead")
        return _create_default_office(size)


def _create_default_office(size: int = 50) -> np.ndarray:
    """
    Create a simple office with rooms and hallways.
    """
    grid = np.zeros((size, size), dtype=np.int8)
    
    # Outer walls

    
    # Horizontal hallway (middle)
    hallway_y = size // 2
    
    # Vertical hallway (middle)
    hallway_x = size // 2
    
    # Room dividers (with doorways)
    for y in range(1, size-1):
        if y % 10 != 5:  # Leave doorways every 10 cells
            if y != hallway_y:  # Don't block main hallway
                grid[y, hallway_x] = -1
    
    for x in range(1, size-1):
        if x % 10 != 5:  # Leave doorways
            if x != hallway_x:
                grid[hallway_y, x] = -1
    
    # Add some furniture/obstacles in rooms
    for _ in range(int(size * 0.3)):  # 30% of size
        x = random.randint(2, size-3)
        y = random.randint(2, size-3)
        # Don't place in hallways
        if abs(x - hallway_x) > 2 and abs(y - hallway_y) > 2:
            grid[y, x] = -1
    
    return grid


def _create_office_layout(size: int = 50) -> np.ndarray:
    """
    Create a realistic office with conference rooms, cubicles, and corridors.
    """
    grid = np.zeros((size, size), dtype=np.int8)
    
    # Outer walls
    grid[0, :] = -1
    grid[-1, :] = -1
    grid[:, 0] = -1
    grid[:, -1] = -1
    
    # Main corridor (horizontal)
    corridor_y = size // 3
    
    # Conference room (top left)
    grid[5:15, 5:20] = -1  # Walls
    grid[6:14, 6:19] = 0   # Interior
    grid[10, 5] = 0        # Door
    
    # Cubicle area (top right)
    for x in range(25, 45, 5):
        for y in range(5, 20, 5):
            # Small cubicle walls
            grid[y:y+4, x] = -1
            grid[y, x:x+4] = -1
    
    # Break room (bottom left)
    grid[corridor_y+5:corridor_y+15, 5:15] = -1
    grid[corridor_y+6:corridor_y+14, 6:14] = 0
    grid[corridor_y+10, 5] = 0  # Door
    
    # Individual offices (bottom right)
    for x in range(25, 45, 10):
        grid[corridor_y+5:corridor_y+15, x] = -1
        grid[corridor_y+5:corridor_y+15, x+9] = -1
        grid[corridor_y+5, x:x+10] = -1
        grid[corridor_y+14, x:x+10] = -1
        grid[corridor_y+10, x] = 0  # Door
    
    return grid


def _create_cubicle_layout(size: int = 50) -> np.ndarray:
    """
    Create an office with cubicle rows.
    """
    grid = np.zeros((size, size), dtype=np.int8)
    
    # Outer walls
    grid[0, :] = -1
    grid[-1, :] = -1
    grid[:, 0] = -1
    grid[:, -1] = -1
    
    # Create cubicle rows
    cubicle_size = 6
    aisle_width = 4
    
    for y in range(5, size-5, cubicle_size + aisle_width):
        for x in range(5, size-5, cubicle_size + 2):
            # Cubicle walls (3-sided)
            if x + cubicle_size < size - 5:
                grid[y:y+cubicle_size, x] = -1  # Left wall
                grid[y:y+cubicle_size, x+cubicle_size] = -1  # Right wall
                grid[y, x:x+cubicle_size+1] = -1  # Back wall
                # Front is open (no wall)
    
    return grid


def _create_warehouse_layout(size: int = 50) -> np.ndarray:
    """
    Create a warehouse with storage racks.
    """
    grid = np.zeros((size, size), dtype=np.int8)
    
    # Outer walls
    grid[0, :] = -1
    grid[-1, :] = -1
    grid[:, 0] = -1
    grid[:, -1] = -1
    
    # Storage rack rows
    rack_width = 3
    aisle_width = 5
    
    for x in range(5, size-5, rack_width + aisle_width):
        for y in range(5, size-5):
            if y % 15 != 0:  # Leave gaps for cross-aisles
                grid[y, x:x+rack_width] = -1
    
    return grid


def _create_maze_layout(size: int = 50) -> np.ndarray:
    """
    Create a maze-like environment using recursive division.
    """
    grid = np.zeros((size, size), dtype=np.int8)
    
    # Outer walls
    grid[0, :] = -1
    grid[-1, :] = -1
    grid[:, 0] = -1
    grid[:, -1] = -1
    
    def divide(x1, y1, x2, y2, horizontal):
        if x2 - x1 < 4 or y2 - y1 < 4:
            return
        
        if horizontal:
            # Add horizontal wall with gap
            wall_y = random.randint(y1 + 2, y2 - 2)
            gap_x = random.randint(x1 + 1, x2 - 1)
            for x in range(x1, x2):
                if x != gap_x:
                    grid[wall_y, x] = -1
            
            divide(x1, y1, x2, wall_y, not horizontal)
            divide(x1, wall_y, x2, y2, not horizontal)
        else:
            # Add vertical wall with gap
            wall_x = random.randint(x1 + 2, x2 - 2)
            gap_y = random.randint(y1 + 1, y2 - 1)
            for y in range(y1, y2):
                if y != gap_y:
                    grid[y, wall_x] = -1
            
            divide(x1, y1, wall_x, y2, not horizontal)
            divide(wall_x, y1, x2, y2, not horizontal)
    
    # Start recursive division
    divide(1, 1, size-1, size-1, random.choice([True, False]))
    
    return grid


def _create_open_plan_office(size: int = 50) -> np.ndarray:
    """
    Create an open-plan office with scattered desks and meeting areas.
    """
    grid = np.zeros((size, size), dtype=np.int8)
    
    # Outer walls

    # Randomly place desk clusters
    for _ in range(10):
        cx = random.randint(10, size-10)
        cy = random.randint(10, size-10)
        
        # 2x2 desk cluster
        grid[cy:cy+2, cx:cx+2] = -1
    
    # Meeting pods (small enclosed areas)
    for _ in range(3):
        mx = random.randint(10, size-15)
        my = random.randint(10, size-15)
        
        # Small room
        grid[my:my+8, mx] = -1
        grid[my:my+8, mx+7] = -1
        grid[my, mx:mx+8] = -1
        grid[my+7, mx:mx+8] = -1
        grid[my+4, mx] = 0  # Door
    
    return grid


def get_spawn_points(grid: np.ndarray, n_agents: int) -> List[Tuple[int, int]]:
    """
    Find good spawn points for agents in a custom map.
    
    Tries to spread agents out in free space.
    
    Args:
        grid: Map array (0=free, -1=obstacle)
        n_agents: Number of agents to spawn
        
    Returns:
        List of (x, y) spawn positions
    """
    free_cells = list(zip(*np.where(grid == 0)))
    
    if len(free_cells) < n_agents:
        raise ValueError(f"Not enough free space for {n_agents} agents (only {len(free_cells)} free cells)")
    
    # Try to spread agents out
    spawn_points = []
    
    # Start with a random free cell
    first_point = random.choice(free_cells)
    spawn_points.append(first_point)
    free_cells.remove(first_point)
    
    # For each remaining agent, find the cell farthest from existing spawns
    for _ in range(n_agents - 1):
        max_min_dist = -1
        best_cell = None
        
        # Sample some candidates instead of checking all (faster)
        candidates = random.sample(free_cells, min(100, len(free_cells)))
        
        for cell in candidates:
            # Find minimum distance to any existing spawn
            min_dist = min(abs(cell[0] - s[0]) + abs(cell[1] - s[1]) 
                          for s in spawn_points)
            
            if min_dist > max_min_dist:
                max_min_dist = min_dist
                best_cell = cell
        
        spawn_points.append(best_cell)
        free_cells.remove(best_cell)
    
    return spawn_points


# Example office map as ASCII art (for easy editing)
EXAMPLE_OFFICE_MAP = """
##################################################
#                                                #
#  ########    ########    ########    ########  #
#  #      #           #    #      #    #      #  #
#  #      #    #      #    #      #    #      #  #
#  #      #    #      #           #    #      #  #
#  ########    ########    ########    ########  #
#                                                #
#                                                #
#  ########    ########    ########    ########  #
#  #           #      #    #      #    #      #  #
#  #      #    #      #    #      #    #      #  #
#  #      #    #      #    #      #    #      #  #
#  ########    ########    ########    ########  #
#                                                #
#                                                #
##################################################
"""


def parse_ascii_map(ascii_map: str, size: int = 50) -> np.ndarray:
    """
    Convert ASCII art map to grid array.
    
    Args:
        ascii_map: Multi-line string with # for walls, spaces for free
        size: Desired grid size
        
    Returns:
        grid: 2D numpy array
    """
    lines = ascii_map.strip().split('\n')
    grid = np.zeros((size, size), dtype=np.int8)
    
    for y, line in enumerate(lines):
        if y >= size:
            break
        for x, char in enumerate(line):
            if x >= size:
                break
            if char == '#':
                grid[y, x] = -1
    
    return grid
