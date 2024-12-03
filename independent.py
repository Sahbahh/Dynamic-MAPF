from resolve_conflicts import resolve_conflicts
from search_algorithms import search_algorithm
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost

"""
Obstacle dictionary:
 - key, value = timestep, obstacle information: {appearance_timestep, obstacle_location}
 - Appearance timestep is how many timesteps this obstacle will appear for. By precalculating this
   we can add constraints to agents in specific timesteps to avoid said obstacle.
 
 compile_obstacle_dict function:
 
 Inputs:
 - JSON obstacle data
 - timesteps in which obstacles appear
 - number of rows of map
 - number of cols of map
 - max_steps (used to set appearance time of obstacle in case obstacle stay indefinitely without a remove)

 Outputs:
 - dictionary of obstacles: used for adding constraints to agents to make agents avoid obstacles
 
"""
def compile_obstacle_dict(input_data, obstacle_timesteps, rows, cols, max_steps):
    result = dict()
    memo = [[0 for _ in range(cols)] for _ in range(rows)]
    max_obstacle_time = max(obstacle_timesteps)

    for obstacle_time in obstacle_timesteps:
        if str(obstacle_time) in input_data:
            changes = input_data[str(obstacle_time)]

            for obstacle_location in changes.get("add_obstacles", []):
                memo[obstacle_location[0]][obstacle_location[1]] = obstacle_time

            for obstacle_location in changes.get("remove_obstacles", []):
                appear_time = memo[obstacle_location[0]][obstacle_location[1]]
                appearance_timestep = obstacle_time - appear_time #obstacle appear for how many time steps
                new_c = {'appearance_timestep': appearance_timestep,
                         'loc': (obstacle_location[0],obstacle_location[1])}
                if appear_time not in result:
                    result[appear_time] = []
                result.get(appear_time).append(new_c)
                memo[obstacle_location[0]][obstacle_location[1]] = 0 # reset time of removed obstacle

    for r in range(len(memo)):
        for c in range(len(memo[0])):
            appear_time = memo[r][c]
            if 0 < appear_time <= max_obstacle_time:
                appearance_timestep = max_steps - appear_time  # no remove command, so obstacle stay indefinitely until
                                                            # max steps
                new_c = {'appearance_timestep': appearance_timestep,
                         'loc': (r,c)}
                if appear_time not in result:
                    result[appear_time] = []
                result.get(appear_time).append(new_c)

    return result

"""
Replan all agent paths every time we update the constraints.
Constraints added may be due to new obstacles, or agent locations.
"""
def replan(number_agents, single_agent_planner_map, starts, goals, heuristics, constraints):
    result = []
    for i in range(number_agents):  # Find path for each agent
        path = a_star(single_agent_planner_map, starts[i], goals[i], heuristics[i],
                      i, constraints)
        if path is None:
            raise BaseException('No solutions')
        result.append(path)

    print("paths:")
    for p in result:
        print(p)
    return result

"""
Change format of the map to use with single_agent_planner.py
"""
def initialize_single_agent_planner_map(map):
    result = []
    for i in range(len(map)):
        new_row = []
        for j in range(len(map[0])):
            if map[i][j] == 0:
                new_row.append(False) # no obstacle
            elif map[i][j] == 1:
                new_row.append(True) # there is an obstacle/wall
        result.append(new_row)
    return result


def independent_solver(map_grid, starts, goals, algorithm="a_star", dynamic_changes=None):
    """
    Moves agents toward their respective goals step-by-step using the specified algorithm, while ensuring constraints.

    Args:
        map_grid (list): 2D grid representing the map with obstacles (1 for obstacle, 0 for free space).
        starts (list): List of agent starting positions [(row, col), ...].
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

    # for agent in range(number_agents):
    #     result = []
    #
    #     ##############################
    #
    #     for i in range(number_agents):  # Find path for each agent
    #         path = a_star(map_grid, start[i], goals[i], #heuristics[i],
    #                       i, [])
    #         if path is None:
    #             raise BaseException('No solutions')
    #         result.append(path)


    # for i, (agent_location, goal) in enumerate(zip(agents, goals)):
    #     print(f"\nAgent {i + 1}: Current Position: {agent_location}, Goal: {goal}")
    #
    #     # Check if the agent is on a new obstacle
    #     if agent_location in new_obstacles:
    #         print(f"Agent {i + 1} is on a new obstacle at {agent_location}. Relocating...")
    #         new_position = relocate_to_nearest_neighbor(agent_location, map_grid)
    #         print(f"Agent {i + 1} relocated to {new_position}")
    #         updated_positions.append(new_position)
    #         continue
    #
    #     # If the agent is already at its goal, no further action is required
    #     if agent_location == goal:
    #         print(f"Agent {i + 1} has reached its goal.")
    #         updated_positions.append(agent_location)
    #         continue
    #
    #     # Call the appropriate search algorithm to get the next move
    #     next_move = search_algorithm(algorithm, agent_location, goal, map_grid, {})
    #     print(f"Agent {i + 1}: Next Move from Algorithm: {next_move}")
    #
    #     # Validate the returned move
    #     if next_move and is_valid_move(next_move, map_grid):
    #         print(f"Agent {i + 1}: Selected Move: {next_move}")
    #         updated_positions.append(next_move)
    #     else:
    #         print(f"Agent {i + 1}: No valid move. Staying in place.")
    #         updated_positions.append(agent_location)

    # agent_conflicts = get_conflicts(updated_positions)
    # print("AGENT CONFLICTS: ", agent_conflicts)

    # for agent1, agent2, conflict_location in agent_conflicts:
    #     print("AGENT CONFLICT:")
    #     print("Conflict between agent ", agent1 + 1," and agent ", agent2 + 1, " at ", conflict_location)

    # new_positions = resolve_conflicts(algorithm, agent_conflicts, goals, map_grid)

    return updated_positions
