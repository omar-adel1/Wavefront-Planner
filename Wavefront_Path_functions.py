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
    for row in range(len(grid)):
        for column in range(len(grid[0])):
            if grid[row][column] == 2:
                return (row, column)
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
        current_row, current_col = queue.popleft()
        current_val = wavefront[current_row][current_col]
        for row_offset, col_offset in DIRECTIONS:
            next_row = current_row + row_offset
            next_col = current_col + col_offset
            if 0 <= next_row < len(wavefront) and 0 <= next_col < len(wavefront[0]) and wavefront[next_row][next_col] == 0:
                wavefront[next_row][next_col] = current_val + 1
                queue.append((next_row, next_col))

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
    indices = list()  
    indices.append(start)

    current_cell = start

    while wavefront[current_cell[0]][current_cell[1]] != 2:
        current_row, current_col = current_cell
        next_cell, min_val = None, wavefront[r][c]

        for row_offset, col_offset in DIRECTIONS:
            neighbor_row = current_row + row_offset
            neighbor_col = current_col + col_offset
            if 0 <= neighbor_row < len(wavefront) and 0 <= neighbor_col < len(wavefront[0]) and wavefront[neighbor_row][neighbor_col] < min_val and wavefront[neighbor_row][neighbor_col] != 1:
                next_cell, min_val = (neighbor_row, neighbor_col), wavefront[neighbor_row][neighbor_col]

        if not next_cell:
            raise ValueError("No valid path found.")
        
        trajectory.append(next_cell)
        indices.append(next_cell) 
        current_cell = next_cell

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
