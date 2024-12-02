from search_algorithms import search_algorithm

def independent_solver(map_grid, agents, goals, algorithm="a_star", dynamic_changes=None):
    """
    Moves agents toward their respective goals step-by-step using the specified algorithm, while ensuring constraints.

    Args:
        map_grid (list): 2D grid representing the map with obstacles (1 for obstacle, 0 for free space).
        agents (list): List of current agent positions [(row, col), ...].
        goals (list): List of goal positions [(row, col), ...].
        algorithm (str): The search algorithm to use ("a_star", "cbs", etc.).
        dynamic_changes (dict): Optional dynamic changes, such as added obstacles.

    Returns:
        list: Updated list of agent positions [(row, col), ...].
    """
    def is_valid_move(position, grid):
        """Check if a move is valid (within bounds and not an obstacle)."""
        rows, cols = len(grid), len(grid[0])
        r, c = position
        return 0 <= r < rows and 0 <= c < cols and grid[r][c] == 0

    def relocate_to_nearest_neighbor(agent, grid):
        """Find the first valid immediate neighbor to relocate to."""
        r, c = agent
        potential_moves = [
            (r - 1, c),  # Up
            (r + 1, c),  # Down
            (r, c - 1),  # Left
            (r, c + 1)   # Right
        ]
        for move in potential_moves:
            if is_valid_move(move, grid):
                return move
        return agent  # Stay in place if no valid moves exist

    updated_positions = []

    # Handle dynamic changes
    new_obstacles = set(tuple(obstacle) for obstacle in dynamic_changes.get("add_obstacles", [])) if dynamic_changes else set()

    for i, (agent, goal) in enumerate(zip(agents, goals)):
        print(f"Agent {i + 1}: Current Position: {agent}, Goal: {goal}")

        # Check if the agent is on a new obstacle
        if agent in new_obstacles:
            print(f"Agent {i + 1} is on a new obstacle at {agent}. Relocating...")
            new_position = relocate_to_nearest_neighbor(agent, map_grid)
            print(f"Agent {i + 1} relocated to {new_position}")
            updated_positions.append(new_position)
            continue

        # If the agent is already at its goal, no further action is required
        if agent == goal:
            print(f"Agent {i + 1} has reached its goal.")
            updated_positions.append(agent)
            continue

        # Call the appropriate search algorithm to get the next move
        next_move = search_algorithm(algorithm, agent, goal, map_grid, {})
        print(f"Agent {i + 1}: Next Move from Algorithm: {next_move}")

        # Validate the returned move
        if next_move and is_valid_move(next_move, map_grid):
            print(f"Agent {i + 1}: Selected Move: {next_move}")
            updated_positions.append(next_move)
        else:
            print(f"Agent {i + 1}: No valid move. Staying in place.")
            updated_positions.append(agent)

    return updated_positions
