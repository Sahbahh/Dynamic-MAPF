from independent import IndependentSolver
from input_parser import parse_input
from helper_functions import initialize_single_agent_planner_map, \
    compile_obstacle_dict, compile_goals_dict, update_constraints, replan
from map_validator import validate_map  # Import the validator
from single_agent_planner import compute_heuristics
from prioritized import PrioritizedPlanningSolver
from cbs import CBSSolver
from dynamic_map_visualizer import Animation
from functools import reduce
from lns import LNSSolver
import json
import copy

from space_time_a_star import SpaceTimePlanningSolver

"""
DYNAMIC MAPPING FOR MAPF ALGORITHMS
STEPS:
1. Planning all agent paths with a star
2. Planning all agent paths with a star + obstacle constraints
3. Adapt to changing goals with chosen MAPF algorithm

- UI can be paused with a button click

"""

def main():
    """
            DYNAMIC MAPPING FOR MAPF ALGORITHMS
        STEPS:
        1. Planning all agent paths with a star
        2. Planning all agent paths with a star + obstacle constraints
        3. Adapt to changing goals with chosen MAPF algorithm
            - Space-Time A*
            - Prioritized
            - CBS
            - CBS with disjoint splitting
            - Large Neighborhood Search (LNS)

        -> UI animation can be paused with a button click
    """

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
    # compute heuristics for the low-level search
    heuristics = []
    for goal in agent_goals:
        heuristics.append(compute_heuristics(single_agent_planner_map, goal))

    ##########################################
    ## SECTION 1: INDEPENDENT AGENT PLANNER ##
    """
    Agent paths are planned independently with a-star search regardless of obstacles or agent collisions.
    """
    print("starts: ", agent_starts)
    print("goals: ", agent_goals)

    agent_constraints = []
    print("RESULT PATH WITH INDEPENDENT PLANNING")
    solver = IndependentSolver(
        single_agent_planner_map, agent_starts, agent_goals)
    result = solver.find_solution()

    for i,p in enumerate(result):
        print(i,p)

    ########################################################################

    #### SECTION 2: OBSTACLES DETECTION AND AVOIDANCE USING CONSTRAINTS ####
    """
    With this feature, agents will avoid obstacles with added restraints. Agent collision can still happen.
    This feature applies to all MAPF algorithms.
    """

    # time steps of all input data
    input_data_timesteps = sorted(map(int, input_data.keys()))  # Ensure timesteps are processed in order


    max_steps = 100  # Maximum allowed steps to prevent infinite loops

    # record all obstacles creation and how many time steps they last
    obstacle_dictionary = compile_obstacle_dict(input_data, input_data_timesteps, rows, cols, max_steps)

    # Add obstacle constraints
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    for timestep, obstacle_list in obstacle_dictionary.items():
        for obstacle in obstacle_list:
            obstacle_location = obstacle['loc']
            current_time = timestep
            obstacle_appear_time = obstacle['appearance_timestep']

            if obstacle_appear_time == -1:  # permanent obstacle, then give inf constraint
                for agent in range(number_agents):
                    agent_constraints.append({'agent': agent, 'loc': [obstacle_location],
                                              'timestep': current_time, 'type': 'inf-obstacle'})
            else:  # not a permanent obstacle, then add vertex and edge constraints
                for _ in range(
                        obstacle['appearance_timestep']):  # add constraints to agents for as long as the obstacle
                    # exist
                    for agent in range(number_agents):
                        agent_constraints.append({'agent': agent, 'loc': [obstacle_location],
                                                  'timestep': current_time, 'type': 'vertex-obstacle'})
                        start_pos = []
                        for dir in directions:
                            # prevent agent from going into the obstacle from 4 directions
                            pos = (obstacle_location[0] + dir[0], obstacle_location[1] + dir[1])
                            # do not add edge constraints that fall out of map
                            if pos[0] < 0 or pos[0] >= rows or pos[1] < 0 or pos[1] >= cols:
                                continue
                            start_pos.append(pos)
                        for pos in start_pos:
                            agent_constraints.append({'agent': agent, 'loc': [pos, obstacle_location],
                                                      'timestep': current_time + 1, 'type': 'edge-obstacle'})
                    current_time += 1

    # Replan all agent paths using added constraints with a star search.
    print("RESULT PATH WITH OBSTACLE CONSTRAINTS")

    result = replan(number_agents, single_agent_planner_map, agent_starts, agent_goals,
                    heuristics, agent_constraints)

    for i,p in enumerate(result):
        print(i,p)

    ########################################################################

    ######### SECTION 3: AGENT COLLISION DETECTION AND AVOIDANCE ###########
    """
            Implement replanning of agent paths to avoid agent collision
            using our chosen path planning algorithms here:

        1. Space-Time A*
        2. Prioritized
        3. CBS
        4. CBS with disjoint splitting
        5. Large Neighborhood Search (LNS)

    """

    goal_dictionary = compile_goals_dict(input_data, input_data_timesteps)
    result = []

    # Initialize solution with initial goals
    # REMEMBER TO INITIALIZE YOUR AGENT PATHS
    # 1. Create your own algorithm class (eg. prioritized.py)
    # 2. If your algorithm lower level search is a-star, then you can call single_agent_planner.py with your
    # algorithm class. e.g. prioritized.py calls single_agent_planner.py.
    if args.algorithm == "STA*":
        print("***Initialize Space Time A****")
        solver = SpaceTimePlanningSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints, max_steps)
        result = solver.find_solution()
    elif args.algorithm == "Prioritized":
        print("***Initialize Prioritized***")
        solver = PrioritizedPlanningSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution()
   
    elif args.algorithm == "CBS":
        print("***Initialize CBS***")
        solver = CBSSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution()

    elif args.algorithm == "CBS Disjoint":
        print("***Initialize CBS with disjoint splitting***")
        solver = CBSSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution(disjoint=True)

    elif args.algorithm == "LNS":
        print("***Initialize Large Neighborhood Search***")
        solver = LNSSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution()

    # Adapt to changing goals
    starts = copy.deepcopy(agent_starts)
    goals = copy.deepcopy(agent_goals)
    constraints = copy.deepcopy(agent_constraints)
    new_result = []

    sorted_goal_timestep = sorted(goal_dictionary.keys())

    for i in range(len(sorted_goal_timestep)):
        # Extend all paths to the longest path
        max_path = len(reduce(lambda x, y: x if len(x) > len(y) else y, result))
        for j, p in enumerate(result):
            p_len = max_path - len(p)
            if p_len > 0:
                last_pos = result[j][-1]
                for _ in range(p_len):
                    result[j].append(last_pos)

        goal_timestep = sorted_goal_timestep[i]
        goal_list = goal_dictionary.get(goal_timestep)
        update_constraint_time = goal_timestep if i == 0 else goal_timestep - sorted_goal_timestep[i - 1]

        update_constraints(goal_timestep - 1, update_constraint_time, result, goal_list, starts, goals, constraints)

        if args.algorithm == "STA*":
            solver = SpaceTimePlanningSolver(single_agent_planner_map, starts, goals, constraints,
                                             max_steps)
            new_result = solver.find_solution()
        elif args.algorithm == "Prioritized":
            solver = PrioritizedPlanningSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution()
        elif args.algorithm == "CBS":
            solver = CBSSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution(disjoint=False)
        elif args.algorithm == "CBS Disjoint":
            solver = CBSSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution(disjoint=True)
        elif args.algorithm == "LNS":
            solver = LNSSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution()

        for j, _ in enumerate(result):
            if j < len(new_result):
                result[j] = result[j][:goal_timestep - 1] + new_result[j]
            else:
                print(f"Warning: No new path found for agent {j}. Keeping the original path.")

    print("DYNAMIC OBSTACLES + DYNAMIC GOALS + COLLISION DETECTION AGENT PATHS")
    for i, p in enumerate(result):
        print(i, p)

    #############################################################

    ################ VISUALIZE AGENT PATHS ######################

    # Animation class is in dynamic_map_visualizer.py
    # goal_dictionary = []
    animation = Animation(single_agent_planner_map, agent_starts, agent_goals, result,
                          obstacle_dictionary, goal_dictionary)
    animation.show()
if __name__ == "__main__":
    main()
