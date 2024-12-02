# independent.py

def independent_solver(map_grid, agents, goals):
    """
    Moves agents toward their respective goals step-by-step.
    We won't use this in our project and this is only used to see if the agents actually move in the map

    Args:
        map_grid (list): 2D grid representing the map with obstacles (1 for obstacle, 0 for free space).
        agents (list): List of current agent positions [(row, col), ...].
        goals (list): List of goal positions [(row, col), ...].

    Returns:
        list: Updated list of agent positions [(row, col), ...].
    """
    def is_valid_move(position, grid):
        """Check if a move is valid (within bounds and not an obstacle)."""
        rows, cols = len(grid), len(grid[0])
        r, c = position
        return 0 <= r < rows and 0 <= c < cols and grid[r][c] == 0

    updated_positions = []
    for agent, goal in zip(agents, goals):
        if agent == goal:
            updated_positions.append(agent)  # Agent already at the goal
            continue

        # Calculate potential moves
        r, c = agent
        potential_moves = [
            (r - 1, c),  # Move up
            (r + 1, c),  # Move down
            (r, c - 1),  # Move left
            (r, c + 1)   # Move right
        ]

        # Filter valid moves
        valid_moves = [move for move in potential_moves if is_valid_move(move, map_grid)]

        # Select the move that reduces Manhattan distance
        if valid_moves:
            best_move = min(valid_moves, key=lambda move: abs(move[0] - goal[0]) + abs(move[1] - goal[1]))
            updated_positions.append(best_move)
        else:
            # No valid moves; stay in place
            updated_positions.append(agent)

    return updated_positions
