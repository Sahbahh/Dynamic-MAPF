from input_parser import parse_input
from helper_functions import initialize_single_agent_planner_map, replan, compile_obstacle_dict, compile_goals_dict, update_constraints
from map_validator import validate_map
from single_agent_planner import compute_heuristics, get_sum_of_cost
import matplotlib.pyplot as plt
from prioritized import PrioritizedPlanningSolver
from cbs import CBSSolver
from dynamic_map_visualizer import Animation
from functools import reduce
from lns import LNSSolver
import json
import copy
import argparse


"""
This is the implementation for running all the algorithms

"""

def convert_to_rc(position, rows, cols):
    # position could be [row, col] or a single integer node ID
    if isinstance(position, int):
        # Convert single integer node ID into (row, col)
        return (position // cols, position % cols)
    elif isinstance(position, list) and len(position) == 2:
        # Already [row, col], just make it a tuple
        return (position[0], position[1])
    else:
        raise ValueError(f"Agent position {position} is not in a recognized format.")


def run_single_algorithm(input_file, algorithm_name):
    # Load and validate input data
    with open(input_file, "r") as f:
        data = json.load(f)
    if not validate_map(data):
        print("Map validation failed. Exiting.")
        return None

    # Parse the input and initialize variables
    map_dimensions, agents_data, input_data = parse_input(input_file)
    rows, cols = map_dimensions

    agent_starts = []
    agent_goals = []
    for agent in agents_data:
        start_rc = convert_to_rc(agent["start"], rows, cols)
        goal_rc = convert_to_rc(agent["goal"], rows, cols)
        agent_starts.append(start_rc)
        agent_goals.append(goal_rc)

    map_grid = [[0 for _ in range(cols)] for _ in range(rows)]
    number_agents = len(agent_starts)
    single_agent_planner_map = initialize_single_agent_planner_map(map_grid)

    heuristics = [compute_heuristics(single_agent_planner_map, g) for g in agent_goals]

    # Initial planning
    agent_constraints = []
    replan(number_agents, single_agent_planner_map, agent_starts, agent_goals, heuristics, agent_constraints)

    input_data_timesteps = sorted(map(int, input_data.keys()))
    max_steps = 100

    # Compile obstacles and goals dictionaries
    obstacle_dictionary = compile_obstacle_dict(input_data, input_data_timesteps, rows, cols, max_steps)
    goal_dictionary = compile_goals_dict(input_data, input_data_timesteps)

    # Add obstacle constraints
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    for timestep, obstacle_list in obstacle_dictionary.items():
        for obstacle in obstacle_list:
            obstacle_location = obstacle['loc']
            current_time = timestep
            for _ in range(obstacle['appearance_timestep']):
                for agent in range(number_agents):
                    agent_constraints.append({'agent': agent, 'loc': obstacle_location,
                                              'timestep': current_time, 'type': 'vertex'})
                    start_pos = []
                    for dir in directions:
                        pos = (obstacle_location[0] + dir[0], obstacle_location[1] + dir[1])
                        if pos[0] < 0 or pos[0] >= rows or pos[1] < 0 or pos[1] >= cols:
                            continue
                        start_pos.append(pos)
                    for pos in start_pos:
                        agent_constraints.append({'agent': agent, 'loc': [pos, obstacle_location],
                                                  'timestep': current_time, 'type': 'edge'})
                current_time += 1

    # Run the chosen MAPF algorithm
    if algorithm_name == "Prioritized":
        solver = PrioritizedPlanningSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution()
    elif algorithm_name == "CBS":
        solver = CBSSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution(disjoint=False)
    elif algorithm_name == "CBS Disjoint":
        solver = CBSSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution(disjoint=True)
    elif algorithm_name == "LNS":
        solver = LNSSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints)
        result = solver.find_solution()
    else:
        print(f"Algorithm {algorithm_name} not implemented in run_all_algorithms.")
        return None

    starts = copy.deepcopy(agent_starts)
    goals = copy.deepcopy(agent_goals)
    constraints = copy.deepcopy(agent_constraints)
    new_result = []

    for goal_timestep, goal_list in goal_dictionary.items():
        # Extend all paths to the longest path
        max_path = len(reduce(lambda x, y: x if len(x) > len(y) else y, result))
        for i, p in enumerate(result):
            p_len = max_path - len(p)
            if p_len > 0:
                last_pos = result[i][-1]
                for j in range(p_len):
                    result[i].append(last_pos)

        update_constraints(goal_timestep - 1, result, goal_list, starts, goals, constraints)

        if algorithm_name == "Prioritized":
            solver = PrioritizedPlanningSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution()
        elif algorithm_name == "CBS":
            solver = CBSSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution(disjoint=False)
        elif algorithm_name == "CBS Disjoint":
            solver = CBSSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution(disjoint=True)
        elif algorithm_name == "LNS":
            solver = LNSSolver(single_agent_planner_map, starts, goals, constraints)
            new_result = solver.find_solution()

        for i, _ in enumerate(result):
            if i < len(new_result):
                result[i] = result[i][:goal_timestep - 1] + new_result[i]
            else:
                print(f"Warning: No new path found for agent {i}. Keeping the original path.")

    # Visualize the final result for this algorithm
    animation = Animation(single_agent_planner_map, agent_starts, agent_goals, result,
                          obstacle_dictionary, goal_dictionary, algorithm_name=algorithm_name)
    animation.show()

    return result, solver

def main():
    parser = argparse.ArgumentParser(description="Run all algorithms for MAPF")
    parser.add_argument("input_file", type=str, help="Path to the input JSON file")
    args = parser.parse_args()

    algorithms_to_run = ["Prioritized", "CBS", "CBS Disjoint", "LNS"]
    summary_data = []

    for alg in algorithms_to_run:
        print(f"\n--- Running {alg} ---")
        result, solver = run_single_algorithm(args.input_file, alg)
        sum_of_cost = get_sum_of_cost(result)
        expanded = getattr(solver, 'num_of_expanded', 0)
        generated = getattr(solver, 'num_of_generated', 0)
        summary_data.append([alg, sum_of_cost, expanded, generated])

    # Display summary table
    fig, ax = plt.subplots(figsize=(6, 3))
    fig.suptitle("Comparison of the Algorithms")
    columns = ["Algorithm", "Sum of Cost", "Expanded", "Generated"]
    ax.axis('tight')
    ax.axis('off')
    table = ax.table(cellText=summary_data, colLabels=columns, loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.auto_set_column_width(col=list(range(len(columns))))
    plt.show()

if __name__ == "__main__":
    main()
