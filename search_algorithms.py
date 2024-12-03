from heapq import heappop, heappush


def search_algorithm(name, agent, goal, grid, constraints):
    """
    Calls the appropriate search algorithm.

    Args:
        name (str): Algorithm name (e.g., "a_star", "cbs").
        agent (tuple): Current position of the agent.
        goal (tuple): Goal position for the agent.
        grid (list): The map grid (2D list with 0 for free, 1 for obstacles).
        constraints (dict): Constraints like occupied positions or path-crossing prevention.

    Returns:
        tuple: Next position for the agent or None if no valid move exists.
    """
    if name == "a_star":
        return a_star(agent, goal, grid, constraints)
    elif name == "cbs":
        return cbs(agent, goal, grid, constraints)
    else:
        raise ValueError(f"Unknown algorithm: {name}")

from heapq import heappop, heappush

from heapq import heappop, heappush

def a_star(agent, goal, grid, constraints):
    """
    A* algorithm to find the shortest path from agent to goal while avoiding obstacles.

    Args:
        agent (tuple): Starting position of the agent (row, col).
        goal (tuple): Goal position of the agent (row, col).
        grid (list): 2D map grid (0 for free, 1 for obstacle).
        constraints (dict): Constraints like future positions.

    Returns:
        tuple: The next position (row, col) to move to, or None if no valid path exists.
    """
    from heapq import heappop, heappush

    rows, cols = len(grid), len(grid[0])

    def is_valid_move(position):
        """Check if a move is valid and within the grid."""
        r, c = position
        if not (0 <= r < rows and 0 <= c < cols):  # Within bounds
            return False
        if grid[r][c] == 1:  # Not an obstacle
            return False
        if position in constraints.get("future_positions", set()):  # Not occupied
            return False
        return True

    def get_neighbors(position):
        """Generate cardinal (non-diagonal) neighbors of a position."""
        r, c = position
        return [
            (r - 1, c),  # Up
            (r + 1, c),  # Down
            (r, c - 1),  # Left
            (r, c + 1)   # Right
        ]

    def heuristic(pos):
        """Manhattan distance heuristic."""
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    # Priority queue for A* search
    open_set = []
    heappush(open_set, (0 + heuristic(agent), 0, agent))  # (f-score, g-score, position)
    came_from = {}
    g_score = {agent: 0}

    while open_set:
        _, current_g, current = heappop(open_set)

        # If we reached the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path[0] if len(path) > 1 else path[0]

        for neighbor in get_neighbors(current):
            if not is_valid_move(neighbor):
                continue

            tentative_g = current_g + 1  # All moves have cost 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor)
                heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current

    # If no path found
    return None



def cbs(agent, goal, grid, constraints):
    """Conflict-Based Search (CBS) placeholder."""
    print(f"CBS called for Agent at {agent} with goal {goal}.")
    return None  # Placeholder: Implement CBS logic here
