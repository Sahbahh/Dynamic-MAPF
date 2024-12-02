# map_validator.py

def validate_map(input_data):
    """
    Validate the input map for consistency and conflicts.

    Args:
        input_data (dict): The map data loaded from the input JSON file.

    Returns:
        bool: True if the map is valid, False otherwise.
    """
    map_dimensions = input_data["map_dimensions"]
    agents = input_data["agents"]
    timesteps = input_data["timesteps"]

    rows, cols = map_dimensions

    # Initialize the grid
    map_grid = [[0 for _ in range(cols)] for _ in range(rows)]  # 0: Free, 1: Obstacle

    # Initialize agent goals
    agent_goals = [tuple(agent["goal"]) for agent in agents]

    def is_within_bounds(cell):
        """Check if a cell is within map boundaries."""
        r, c = cell
        return 0 <= r < rows and 0 <= c < cols

    # Validate agent start and goal positions
    for idx, agent in enumerate(agents):
        start, goal = tuple(agent["start"]), tuple(agent["goal"])
        if not (is_within_bounds(start) and is_within_bounds(goal)):
            print(f"Error: Agent {idx + 1} start {start} or goal {goal} is out of bounds.")
            return False

    # Process each timestep
    for t, changes in timesteps.items():
        t = int(t)
        # Apply obstacle additions
        for pos in changes.get("add_obstacles", []):
            if not is_within_bounds(pos):
                print(f"Error: Obstacle at {pos} in timestep {t} is out of bounds.")
                return False
            map_grid[pos[0]][pos[1]] = 1  # Add obstacle

        # Apply obstacle removals
        for pos in changes.get("remove_obstacles", []):
            if not is_within_bounds(pos):
                print(f"Error: Obstacle removal at {pos} in timestep {t} is out of bounds.")
                return False
            map_grid[pos[0]][pos[1]] = 0  # Remove obstacle

        # Apply goal changes
        for idx, new_goal in enumerate(changes.get("change_goals", [])):
            if idx < len(agent_goals):
                if not is_within_bounds(new_goal):
                    print(f"Error: New goal {new_goal} for Agent {idx + 1} in timestep {t} is out of bounds.")
                    return False
                agent_goals[idx] = tuple(new_goal)

        # Validate goals against obstacles
        for idx, goal in enumerate(agent_goals):
            if map_grid[goal[0]][goal[1]] == 1:
                print(f"Error: Agent {idx + 1} goal {goal} overlaps with an obstacle at timestep {t}.")
                return False

    print("Map is valid.")
    return True


if __name__ == "__main__":
    import argparse
    import json

    # Set up argument parser
    parser = argparse.ArgumentParser(description="Validate a dynamic map input file.")
    parser.add_argument("input_file", type=str, help="Path to the input JSON file")
    args = parser.parse_args()

    # Load the input file
    with open(args.input_file, "r") as f:
        input_data = json.load(f)

    # Validate the map
    is_valid = validate_map(input_data)
    if not is_valid:
        print("Validation failed.")
    else:
        print("Validation passed.")
