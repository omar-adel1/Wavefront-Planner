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

# ======= Data Handling =======
def load_mat_file(file) -> np.ndarray:
    """
    Args:
        file: The uploaded .mat file containing the environment map.

    Returns:
        np.ndarray: A NumPy array representing the 2D map where:
            - 0 represents free space
            - 1 represents obstacles
            - 2 represents the goal position
    """
    mat_data = sio.loadmat(file)
    map_mat = np.squeeze(mat_data['map']).astype(int) 
    return map_mat

# ======= Wavefront Algorithm =======
def find_goal_cell(grid: List[List[int]]) -> Tuple[int, int]:
    """
    Args:
        grid (List[List[int]]): A 2D list representing the environment map where:
            - 0 represents free space
            - 1 represents obstacles
            - 2 represents the goal position

    Returns:
        Tuple[int, int]: A tuple (row, column) indicating the position of the goal cell.
    """
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] == 2:
                return (r, c)
    raise ValueError("Goal not found in the map (a cell with value 2 is required).")

def propagate_wavefront(grid: List[List[int]], goal: Tuple[int, int]) -> List[List[int]]:
    """
    Args:
        grid (List[List[int]]): A 2D list representing the environment map where:
            - 0 represents free space
            - 1 represents obstacles
            - 2 represents the goal position.
        goal (Tuple[int, int]): Coordinates (row, column) of the goal cell.

    Returns:
        List[List[int]]: A modified grid where each free cell (0) is assigned a value
        representing the shortest number of steps to reach the goal.
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
        wavefront (List[List[int]]): A 2D grid with wavefront propagation values.
        start (Tuple[int, int]): Coordinates (row, column) of the starting position.

    Returns:
        List[Tuple[int, int]]: A list of (row, column) coordinates representing the optimal path.
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
        indices.append(next_cell)  # Store visited point
        current = next_cell

    return trajectory

def planner(map_grid: List[List[int]], start_row: int, start_col: int) -> Tuple[List[List[int]], List[Tuple[int, int]]]:
    """
    Args:
        map_grid (List[List[int]]): A 2D grid representing the environment where:
            - 0 represents free space
            - 1 represents obstacles
            - 2 represents the goal position
        start_row (int): Row index of the starting position.
        start_col (int): Column index of the starting position.

    Returns:
        Tuple[List[List[int]], List[Tuple[int, int]]]:
            - The wavefront grid with propagated distance values.
            - The optimal trajectory as a list of (row, column) coordinates.
    """
    goal = find_goal_cell(map_grid)
    wavefront_grid = propagate_wavefront(map_grid, goal)
    trajectory = extract_optimal_trajectory(wavefront_grid, (start_row, start_col))
    return wavefront_grid, trajectory

# ======= Plotting Functions =======
def plot_map(map_grid, title="Map"):
    """
    Args:
        map_grid (List[List[int]]): A 2D list representing the environment where:
            - 0 represents free space
            - 1 represents obstacles
            - 2 represents the goal position
        title (str): Title of the plot (default is "Map").

    Returns:
        plt: The generated plot object.
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
        value_map (List[List[int]]): A 2D list representing the wavefront value map.
        trajectory (List[Tuple[int, int]]): A list of (row, column) coordinates forming the optimal path.

    Returns:
        plt: The generated plot object.
    """
    value_map_np = np.array(value_map)
    plt.figure(figsize=(6, 6))
    plt.imshow(value_map_np, cmap='viridis', origin='upper')
    plt.colorbar(label="Wavefront Value")

    traj_rows, traj_cols = zip(*trajectory)
    plt.plot(traj_cols, traj_rows, color='red', linewidth=2, marker='o', markersize=5, label="Trajectory")
    
    plt.scatter(traj_cols[0], traj_rows[0], c='green', s=100, label="Start")
    plt.scatter(traj_cols[-1], traj_rows[-1], c='blue', s=100, label="Goal")

    plt.title("Wavefront Planner: Value Map and Optimal Trajectory")
    plt.legend()
    plt.xlabel("Column")
    plt.ylabel("Row")
    return plt
