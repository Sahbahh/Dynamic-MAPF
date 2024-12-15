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

from space_time_a_star import SpaceTimePlanningSolver

"""
This is the implementation for running all the algorithms.
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

    agent_constraints = []

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
    if algorithm_name == "STA*":
        solver = SpaceTimePlanningSolver(single_agent_planner_map, agent_starts, agent_goals, agent_constraints,
                                         max_steps)
        result = solver.find_solution()
    elif algorithm_name == "Prioritized":
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

    # Initialize cumulative expansions and generations
    expansions_cumulative = getattr(solver, 'num_of_expanded', 0)
    generated_cumulative = getattr(solver, 'num_of_generated', 0)

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

        if algorithm_name == "STA*":
            solver = SpaceTimePlanningSolver(single_agent_planner_map, starts, goals, constraints,
                                                 max_steps)
            new_result = solver.find_solution()
        elif algorithm_name == "Prioritized":
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

        expansions_cumulative += getattr(solver, 'num_of_expanded', 0)
        generated_cumulative += getattr(solver, 'num_of_generated', 0)

        for i, _ in enumerate(result):
            if i < len(new_result):
                result[i] = result[i][:goal_timestep - 1] + new_result[i]
            else:
                print(f"Warning: No new path found for agent {i}. Keeping the original path.")

    # After all dynamic changes, create a dummy solver-like object to return
    class FinalSolverStats:
        pass

    final_solver_stats = FinalSolverStats()
    final_solver_stats.num_of_expanded = expansions_cumulative
    final_solver_stats.num_of_generated = generated_cumulative

    # Visualization
    animation = Animation(single_agent_planner_map, agent_starts, agent_goals, result,
                          obstacle_dictionary, goal_dictionary, algorithm_name=algorithm_name)
    animation.show()

    return result, final_solver_stats


def main():
    parser = argparse.ArgumentParser(description="Run all algorithms for MAPF")
    parser.add_argument("input_file", type=str, help="Path to the input JSON file")
    args = parser.parse_args()

    # Load the input once to compute general info
    with open(args.input_file, "r") as f:
        data = json.load(f)

    map_dimensions, agents_data, input_data = parse_input(args.input_file)
    number_of_agents = len(agents_data)

    # Compute final number of obstacles and frequency of changes
    obstacles_set = set()
    goal_changes_count = 0
    obstacle_changes_count = 0

    for t, changes in input_data.items():
        added = changes.get('add_obstacles', [])
        removed = changes.get('remove_obstacles', [])
        # Count goal changes if present
        if 'change_goals' in changes and changes['change_goals']:
            goal_changes_count += 1
        # Count obstacle changes if any additions or removals
        if added or removed:
            obstacle_changes_count += 1

        # Apply obstacle changes to keep track of final obstacle set
        for pos in added:
            obstacles_set.add(tuple(pos))
        for pos in removed:
            if tuple(pos) in obstacles_set:
                obstacles_set.remove(tuple(pos))

    number_of_obstacles = len(obstacles_set)


    algorithms_to_run = ["STA*","Prioritized", "CBS", "CBS Disjoint", "LNS"]
    summary_data = []

    for alg in algorithms_to_run:
        print(f"\n--- Running {alg} ---")
        result, solver = run_single_algorithm(args.input_file, alg)
        sum_of_cost = get_sum_of_cost(result)
        expanded = getattr(solver, 'num_of_expanded', 0)
        generated = getattr(solver, 'num_of_generated', 0)
        summary_data.append([alg, sum_of_cost, expanded, generated])


    #################################
    # Plot the comparison figure
    #################################

    # Display summary table
    fig = plt.figure(figsize=(10, 5))
    
    # map info row
    ax_info = fig.add_subplot(211) 
    ax_info.set_title("Comparison of the Algorithms", fontsize=16, pad=1)
    ax_info.axis('off')
    
    ax_info = plt.gca()
    ax_info.set_position((0, 0, 1, 0.9)) 


    row_data = [[
        f"Map: {map_dimensions[0]}x{map_dimensions[1]}",
        f"#Agents: {number_of_agents}",
        f"#Obstacles: {number_of_obstacles}",
        f"Goal Changes: {goal_changes_count}",
        f"Obstacle Changes: {obstacle_changes_count}"
    ]]

    table_info = ax_info.table(cellText=row_data, loc='top', cellLoc='center', bbox=[0, 0.8, 1, 0.15])

    for key, cell in table_info.get_celld().items():
        cell.set_fontsize(12)
        cell.set_edgecolor('white')
        cell.set_facecolor("#a2d2ff")

    table_info.auto_set_font_size(False)

    # Same width for all the cells
    rows = len(row_data)
    cols = len(row_data[0])
    cell_width = 1 / cols

    for row in range(rows):
        for col in range(cols):
            table_info[(row, col)].set_width(cell_width)

    # Axis for the summary table
    ax_main = fig.add_subplot(212)
    ax_main.axis('off')
    ax_main = plt.gca()




    columns = ["Algorithm", "Sum of Cost", "Expanded", "Generated"]
    table_main = ax_main.table(cellText=summary_data, colLabels=columns, loc='center',bbox=[0, 0.65, 1, 0.95])

    # Style the main table
    header_color = "#cef4ff"
    row_colors = ["#f3cfce", "#f1adb9", "#f09caf", "#ee839f", "#e05780"]

    for key, cell in table_main.get_celld().items():
        if key[0] == 0:  # Header row
            cell.set_facecolor(header_color)
        elif key[0] > 0:
            cell.set_facecolor(row_colors[(key[0]-1) % len(row_colors)])
            
    table_main.auto_set_font_size(False)
    table_main.set_fontsize(12)

    for row in range(rows):
        for col in range(cols):
            table_info[(row, col)].set_width(cell_width)
    
    plt.show()

if __name__ == "__main__":
    main()
