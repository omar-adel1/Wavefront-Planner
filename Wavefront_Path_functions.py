import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
from collections import deque
from typing import List, Tuple

DIRECTIONS = [
    (-1,  0),  # Up
    ( 0,  1),  # Right
    ( 1,  0),  # Down
    ( 0, -1),  # Left
    (-1,  1),  # Upper Right
    ( 1,  1),  # Lower Right
    ( 1, -1),  # Lower Left
    (-1, -1)   # Upper Left
]

# Data Handling 
def load_mat_file(file) -> np.ndarray:
    """
    Args:
        .mat file (maze)

    Returns:
        0= free space
        1= obstacles
        2= end goal position
    """
    mat_data = sio.loadmat(file)
    map_mat = np.squeeze(mat_data['map']).astype(int) 
    return map_mat

# Wavefront Algorithm 
def find_goal_cell(grid: List[List[int]]) -> Tuple[int, int]:
    """
    Args:
        2D list =map (filled with 0, 1, 2)=empty, etc

    Returns:
        tuple (row, column)= position of goal cell
    """
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] == 2:
                return (r, c)
    raise ValueError("Goal not found in the map (a cell with value 2 is required).")

def propagate_wavefront(grid: List[List[int]], goal: Tuple[int, int]) -> List[List[int]]:
    """
    Args:
        grid 2d list (0, 1, 2)=empty, etc
        goal tuple

    Returns:
        grid where each (0/empty space)= assigned value 
        of shortest number of steps to reach goal
    """
    wavefront = [row[:] for row in grid]

    queue = deque([goal])

    while queue:
        r, c = queue.popleft()
        current_val = wavefront[r][c]
        for dr, dc in DIRECTIONS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < len(wavefront) and 0 <= nc < len(wavefront[0]) and wavefront[nr][nc] == 0:
                wavefront[nr][nc] = current_val + 1
                queue.append((nr, nc))

    return wavefront

def extract_optimal_trajectory(wavefront: List[List[int]], start: Tuple[int, int]) -> List[Tuple[int, int]]:   
    """
    Args:
        grid (shortest number of steps)
        start position tuple (row, column)

    Returns:
        list of (row, column) of optimal path.
    """ 
    if wavefront[start[0]][start[1]] in {0, 1}:
        raise ValueError("Start position is unreachable.")

    trajectory = [start]
    indices = list()  # Store visited indices
    indices.append(start)

    current = start

    while wavefront[current[0]][current[1]] != 2:
        r, c = current
        next_cell, min_val = None, wavefront[r][c]

        for dr, dc in DIRECTIONS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < len(wavefront) and 0 <= nc < len(wavefront[0]) and wavefront[nr][nc] < min_val and wavefront[nr][nc] != 1:
                next_cell, min_val = (nr, nc), wavefront[nr][nc]

        if not next_cell:
            raise ValueError("No valid path found.")
        
        trajectory.append(next_cell)
        indices.append(next_cell) 
        current = next_cell

    return trajectory

def planner(map_grid: List[List[int]], start_row: int, start_col: int) -> Tuple[List[List[int]], List[Tuple[int, int]]]:
    """
    Args:
        map grid (0,1,2)
        start_row 
        start_col

    Returns:   
        shortest number of paths grid
        optimal path= list of (row, column) coordinates.
    """
    goal = find_goal_cell(map_grid)
    wavefront_grid = propagate_wavefront(map_grid, goal)
    trajectory = extract_optimal_trajectory(wavefront_grid, (start_row, start_col))
    return wavefront_grid, trajectory

# Plotting Functions
def plot_map(map_grid, title="Map"):
    """
    Args:
        map grid (0,1,2)

    Returns:
        generated plot
    """
    plt.figure(figsize=(6, 6))
    plt.imshow(map_grid, cmap='gray', origin='upper')
    plt.colorbar(label="Map Values")
    plt.title(title)
    plt.xlabel("Column")
    plt.ylabel("Row")
    return plt

def plot_trajectory(value_map, trajectory):
    """
    Args:
        shortest number of paths map
        trajectory coordinates list

    Returns:
        trajectory plot
    """
    value_map_np = np.array(value_map)
    plt.figure(figsize=(6, 6))
    plt.imshow(value_map_np, cmap='viridis', origin='upper')
    plt.colorbar(label="Wavefront Value")

    traj_rows, traj_cols = zip(*trajectory)
    plt.plot(traj_cols, traj_rows, color='red', linewidth=2, marker='o', markersize=1.5, label="Trajectory")
    
    plt.scatter(traj_cols[0], traj_rows[0], c='green', s=100, label="Start")
    plt.scatter(traj_cols[-1], traj_rows[-1], c='blue', s=100, label="Goal")

    plt.title("Wavefront Planner: Value Map and Optimal Trajectory")
    plt.legend()
    plt.xlabel("Column")
    plt.ylabel("Row")
    return plt

def get_trajectory_text(trajectory):
    """
    Args:
        list of trajectory points.

    Returns:
        string representation of the trajectory, each point on a new line
    """
    return "\n".join(str(point) for point in trajectory)
