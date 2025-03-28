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
    Loads a MATLAB (.mat) file and extracts the map data.

    Args:
        file: A .mat file containing the maze map.

    Returns:
        np.ndarray: A 2D array where:
            0 = free space,
            1 = obstacles,
            2 = goal position.
    """
    mat_data = sio.loadmat(file)
    map_mat = np.squeeze(mat_data['map']).astype(int) 
    return map_mat

# Wavefront Algorithm 
def find_goal_cell(grid: List[List[int]]) -> Tuple[int, int]:
    """
    Finds the goal cell (value 2) in a 2D map.

    Args:
        grid (List[List[int]]): Map with 0 = free, 1 = obstacles, 2 = goal.

    Returns:
        Tuple[int, int]: (row, column) coordinates of the goal.
    """
    for row in range(len(grid)):
        for column in range(len(grid[0])):
            if grid[row][column] == 2:
                return (row, column)
    # If we don't find a cell with value 2, raise an error.
    raise ValueError("Goal not found in the map (a cell with value 2 is required).")

def propagate_wavefront(grid: List[List[int]], goal: Tuple[int, int]) -> List[List[int]]:
    """
    Uses BFS to propagate the shortest path distances from the goal cell to all reachable empty spaces.
    
    Args:
        grid (List[List[int]]): 2D list representing the map (0 = empty, 1 = obstacle, 2 = goal).
        goal (Tuple[int, int]): Coordinates (row, column) of the goal cell.

    Returns:
        List[List[int]]: Updated grid with shortest step values assigned to empty cells.
    """
    rows, cols = len(grid), len(grid[0])

    # Ensure the goal cell is valid
    if grid[goal[0]][goal[1]] != 2:
        raise ValueError("Goal cell must contain the value 2.")

    queue = deque([goal])

    while queue:
        row, col = queue.popleft()
        current_distance = grid[row][col]

        # Check all neighbors.
        for row_offset, col_offset in DIRECTIONS:
            next_row, next_col = row + row_offset, col + col_offset
            # If the neighbor is within bounds and free, update its distance.
            if 0 <= next_row < rows and 0 <= next_col < cols and grid[next_row][next_col] == 0:
                grid[next_row][next_col] = current_distance + 1
                queue.append((next_row, next_col))

    return grid

def extract_optimal_trajectory(wavefront: List[List[int]], start: Tuple[int, int]) -> List[Tuple[int, int]]:   
    """
    Extracts the optimal path from the start position to the goal based on wavefront values.

    Args:
        wavefront (List[List[int]]): 2D grid with shortest number of steps to goal.
        start (Tuple[int, int]): Starting position (row, column).

    Returns:
        List[Tuple[int, int]]: Ordered list of (row, column) positions representing the optimal path.
    """ 
    if wavefront[start[0]][start[1]] in {0, 1}:
        raise ValueError("Start position is unreachable.")

    trajectory = [start]
    indices = list()  
    indices.append(start)

    current_cell = start

    while wavefront[current_cell[0]][current_cell[1]] != 2:
        current_row, current_col = current_cell
        next_cell, min_val = None, wavefront[current_row][current_col]

        # Check all directions for the best next step.
        for row_offset, col_offset in DIRECTIONS:
            neighbor_row = current_row + row_offset
            neighbor_col = current_col + col_offset
            # Only consider valid neighbors that are inside the grid and have a lower value
            if 0 <= neighbor_row < len(wavefront) and 0 <= neighbor_col < len(wavefront[0]) and wavefront[neighbor_row][neighbor_col] < min_val and wavefront[neighbor_row][neighbor_col] != 1:
                next_cell, min_val = (neighbor_row, neighbor_col), wavefront[neighbor_row][neighbor_col]

        # If no neighbor with a lower value is found, there is no valid path.
        if not next_cell:
            raise ValueError("No valid path found.")
        
        trajectory.append(next_cell)
        indices.append(next_cell) 
        current_cell = next_cell

    return trajectory

def planner(map_grid: List[List[int]], start_row: int, start_col: int) -> Tuple[List[List[int]], List[Tuple[int, int]]]:
    """
    Creates a wavefront grid and computes the optimal path from the given start point to the goal.

    Args:
        map_grid (List[List[int]]): 2D grid representing the map (0 = free space, 1 = obstacle, 2 = goal).
        start_row (int): The row index for the starting position.
        start_col (int): The column index for the starting position.

    Returns:
        Tuple[List[List[int]], List[Tuple[int, int]]]: 
            - The updated grid with wavefront values showing the shortest number of steps from the goal.
            - The optimal path as a list of (row, column) coordinates.
    """
    
    # Locate the goal (cell with value 2) in the map.
    goal = find_goal_cell(map_grid)
    # Create a grid where each free cell shows how many steps it takes to get to the goal.
    wavefront_grid = propagate_wavefront(map_grid, goal)
    # Starting from our position, follow the decreasing numbers to trace the best path.
    trajectory = extract_optimal_trajectory(wavefront_grid, (start_row, start_col))
    return wavefront_grid, trajectory

# Plotting Functions
def plot_map(map_grid, title="Map"):
    """
    Create a plot of the map.

    Args:
        map_grid: 2D array (with values 0, 1, 2) representing the map.
        title (str): Title of the plot (default "Map").

    Returns:
        The matplotlib.pyplot object with the generated plot.
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
    Create a plot showing the wavefront values and the optimal trajectory.

    Args:
        value_map: 2D grid with shortest step values (wavefront).
        trajectory: List of (row, column) coordinates for the optimal path.

    Returns:
        The matplotlib.pyplot object with the generated plot.
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
    Convert a list of trajectory points into a formatted string, with each point on a new line.

    Args:
        trajectory (list): List of trajectory points.

    Returns:
        str: A string representation of the trajectory, with each point on a separate line.
    """
    return "\n".join(str(point) for point in trajectory)
