def independent_solver(map_grid, agents, goals, dynamic_changes=None):
    """
    Moves agents toward their respective goals step-by-step, accounting for dynamic obstacles.

    Args:
        map_grid (list): 2D grid representing the map with obstacles (1 for obstacle, 0 for free space).
        agents (list): List of current agent positions [(row, col), ...].
        goals (list): List of goal positions [(row, col), ...].
        dynamic_changes (dict): Optional dynamic changes, such as added obstacles.

    Returns:
        list: Updated list of agent positions [(row, col), ...].
    """
    def is_valid_move(position, grid):
        """Check if a move is valid (within bounds and not an obstacle)."""
        rows, cols = len(grid), len(grid[0])
        r, c = position
        return 0 <= r < rows and 0 <= c < cols and grid[r][c] == 0

    def heuristic(position, goal, prev_positions):
        """Heuristic function to evaluate moves."""
        distance = abs(position[0] - goal[0]) + abs(position[1] - goal[1])  # Manhattan distance
        penalty = 1 if position in prev_positions else 0  # Penalize moves to recently visited positions
        return distance + penalty

    updated_positions = []
    prev_positions = set()  # Track recently visited positions for all agents

    # Extract newly added obstacles, if any
    new_obstacles = set(tuple(obstacle) for obstacle in dynamic_changes.get("add_obstacles", [])) if dynamic_changes else set()

    for i, (agent, goal) in enumerate(zip(agents, goals)):
        print(f"Agent {i + 1}: Current Position: {agent}, Goal: {goal}")

        # If the agent is on a newly added obstacle, move it to the first valid spot
        if agent in new_obstacles:
            print(f"Agent {i + 1} is on a new obstacle at {agent}! Relocating...")
            r, c = agent
            potential_moves = [
                (r - 1, c),  # Move up
                (r + 1, c),  # Move down
                (r, c - 1),  # Move left
                (r, c + 1)   # Move right
            ]
            valid_moves = [move for move in potential_moves if is_valid_move(move, map_grid)]

            if valid_moves:
                # Move agent to the first valid cell
                first_valid_move = valid_moves[0]
                print(f"Agent {i + 1} moved to avoid obstacle: {first_valid_move}")
                updated_positions.append(first_valid_move)
                prev_positions.add(agent)  # Update previous positions
                continue
            else:
                print(f"Agent {i + 1} has no valid moves. Staying in place.")
                updated_positions.append(agent)
                continue

        # If the agent is already at its goal, no further action is required
        if agent == goal:
            print(f"Agent {i + 1} has reached its goal.")
            updated_positions.append(agent)
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

        print(f"Agent {i + 1}: Valid Moves: {valid_moves}")

        if valid_moves:
            # Select the best move based on the heuristic
            best_move = min(valid_moves, key=lambda move: heuristic(move, goal, prev_positions))
            print(f"Agent {i + 1}: Selected Move: {best_move}")
            updated_positions.append(best_move)
            prev_positions.add(agent)  # Update previous positions
        else:
            # No valid moves; stay in place
            print(f"Agent {i + 1}: No valid moves. Staying in place.")
            updated_positions.append(agent)

    return updated_positions
