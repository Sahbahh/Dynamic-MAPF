from input_parser import parse_input
from helper_functions import initialize_single_agent_planner_map, replan, \
    compile_obstacle_dict, compile_goals_dict, update_constraints
from map_validator import validate_map  # Import the validator
from single_agent_planner import compute_heuristics
from prioritized import PrioritizedPlanningSolver
from dynamic_map_visualizer import Animation
from functools import reduce
import json
import copy


def main():
    # Set up argument parser
    import argparse
    parser = argparse.ArgumentParser(description="Dynamic Multi-Agent Path Finding (MAPF) Simulation")
    parser.add_argument("input_file", type=str, help="Path to the input JSON file")
    parser.add_argument("--algorithm", type=str, default="a_star", help="Pathfinding algorithm to use (e.g., a_star, cbs)")

    # Parse arguments
    args = parser.parse_args()

    # Load input data
    input_file = args.input_file
    with open(input_file, "r") as f:
        input_data = json.load(f)

    # Validate the map
    print("Validating the map...")
    if not validate_map(input_data):
        print("Map validation failed. Exiting.")
        return

    # Parse the input
    map_dimensions, agents_data, input_data = parse_input(input_file)

    # Extract map dimensions
    rows, cols = map_dimensions

    # Extract agent starts and goals
    agent_starts = [tuple(agent["start"]) for agent in agents_data]
    agent_goals = [tuple(agent["goal"]) for agent in agents_data]
    map_grid = [[0 for _ in range(cols)] for _ in range(rows)]  # 0: Free, 1: Obstacle

    ##############################

    number_agents = len(agent_starts)
    single_agent_planner_map = initialize_single_agent_planner_map(map_grid)

    heuristics = []
    for goal in agent_goals:
        heuristics.append(compute_heuristics(single_agent_planner_map, goal))


    ###############################
    ## INDEPENDENT AGENT PLANNER ##
    """
    Agent paths are planned independent regardless of obstacles or collisions.
    """
    print("starts: ", agent_starts)
    print("goals: ", agent_goals)

    agent_constraints = []
    print("INDEPENDENT AGENT")
    result = replan(number_agents, single_agent_planner_map, agent_starts, agent_goals, heuristics, agent_constraints)

    #############################################################

    #### OBSTACLES DETECTION AND AVOIDANCE USING CONSTRAINTS ####
    """
    With this feature, agents will avoid obstacles with added restraints. Agent collision can still happen.
    """

    # time steps of all input data
    input_data_timesteps = sorted(map(int, input_data.keys()))  # Ensure timesteps are processed in order


    max_steps = 100  # Maximum allowed steps to prevent infinite loops

    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    obstacle_dictionary = compile_obstacle_dict(input_data, input_data_timesteps, rows, cols, max_steps)

    for timestep, obstacle_list in obstacle_dictionary.items():
        for obstacle in obstacle_list:
            obstacle_location = obstacle['loc']
            current_time = timestep
            for _ in range(obstacle['appearance_timestep']):
                for agent in range(number_agents):
                    #vertex constraints
                    agent_constraints.append({'agent': agent, 'loc': obstacle_location,
                                        'timestep': current_time, 'type': 'vertex'})

                    #edge constraints
                    start_pos = []
                    for dir in directions:
                        pos = (obstacle_location[0] + dir[0], obstacle_location[1] + dir[1])
                        if pos[0] < 0 or pos[0] >= rows or pos[1] < 0 or pos[1] >= cols: #out of map
                            continue
                        start_pos.append(pos)
                    for pos in start_pos:
                        agent_constraints.append({'agent': agent, 'loc': [pos,obstacle_location],
                                            'timestep': current_time,'type': 'edge'})

                current_time += 1

    # Replan all agent paths using added constraints with a star.
    print("RESULT PATH WITH OBSTACLE CONSTRAINTS")
    result = replan(number_agents, single_agent_planner_map, agent_starts, agent_goals, heuristics, agent_constraints)

    #############################################################

    ######### AGENT COLLISION DETECTION AND AVOIDANCE ###########
    """
        Implement replanning of agent paths to avoid agent collision 
        using our chosen path planning algorithms here:
        
    1. Prioritized
    2. CBS
    3. CBS with disjoint splitting
    4. Large Neighborhood Search (LNS)
    5. Increased Cost Tree Search (ICTS)
    6. Weighted Dependency Graph (WDG)
    
    """

    goal_dictionary = compile_goals_dict(input_data, input_data_timesteps)

    # Initialize solution with initial goals
    if args.algorithm == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
            result = solver.find_solution()
    elif args.algorithm == "CBS":
            print("***Run CBS***")

    # Adapt to changing goals
    starts = copy.deepcopy(agent_starts)
    goals = copy.deepcopy(agent_goals)
    constraints = copy.deepcopy(agent_constraints)
    new_result = copy.deepcopy(result)
    temp = []

    for goal_timestep, goal_list in goal_dictionary.items():
        # extend all paths to longest path for appending new paths to new goals

        print("GET MAX")
        max_path = len(reduce(lambda x, y: x if len(x) > len(y) else y, new_result))
        for i, p in enumerate(new_result):
            p_len = max_path - len(p)
            if p_len > 0:
                last_pos = new_result[i][-1]
                for j in range(p_len):
                    new_result[i].append(last_pos)

        # change start/goals/constraints to match new timestep
        # goal timestep is 1-index so change to 0-index
        update_constraints(goal_timestep - 1, new_result, goal_list, starts, goals, constraints)

        # add more search algorithms here
        if args.algorithm == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(single_agent_planner_map, starts, goals, constraints)
            temp = solver.find_solution()
            print("NEW RESULT")
            for p in temp:
                print(p)
        elif args.algorithm == "CBS":
            print("***Run CBS***")

        for i, _ in enumerate(result):
            new_result[i] = new_result[i][:goal_timestep - 1] + temp[i]

    print("FINAL RESULT")
    for p in result:
        print(p)


    #############################################################

    ################ VISUALIZE AGENT PATHS ######################

    print("AGENT GOALS")
    print(agent_goals)

    animation = Animation(single_agent_planner_map, agent_starts, agent_goals, new_result,
                          obstacle_dictionary, goal_dictionary)
    animation.show()
if __name__ == "__main__":
    main()
