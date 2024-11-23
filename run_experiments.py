#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def validate_positions(my_map, starts, goals):
    for pos in starts + goals:
        if pos is None:
            raise ValueError(f"Position {pos} is None, which is invalid.")
        if not (0 <= pos[0] < len(my_map) and 0 <= pos[1] < len(my_map[0])):
            raise ValueError(f"Position {pos} is out of map bounds.")
        if my_map[pos[0]][pos[1]]:  # If the position is blocked by an obstacle
            raise ValueError(f"Invalid position {pos} - it is blocked by an obstacle.")


def import_dynamic_mapf_instance(filename):
    with open(filename, 'r') as f:
        dynamic_states = []  # List to store all map states
        current_map = []
        starts = []
        current_goals = []
        timestep = None
        num_agents = 0  # Number of agents

        for line in f:
            line = line.strip()

            if not line:  # Skip empty lines
                continue

            if line.startswith('@') or line.startswith('.'):  # Map rows
                current_map.append([cell == '@' for cell in line])

            elif line.startswith('T ='):  # Timestep indicator
                if timestep is not None:  # Save the previous state before processing a new timestep
                    dynamic_states.append((timestep, current_map, starts, current_goals))
                timestep = int(line.split('=')[1].strip())  # Update timestep
                starts = [None] * num_agents
                current_goals = [None] * num_agents
                current_map = []  # Reset map for the new state

            elif line.isdigit():  # Number of agents
                num_agents = int(line)
                starts = [None] * num_agents
                current_goals = [None] * num_agents

            elif len(line.split()) == 4:  # Agent start/goal positions
                print(f"Parsing agent position: {line}")  # Debugging line
                if num_agents == 0:
                    raise ValueError("Number of agents must be specified before their start/goal positions.")

                sx, sy, gx, gy = map(int, line.split())
                agent_index = len(starts) - starts.count(None)  # Get the next available agent index
                starts[agent_index] = (sx, sy)
                current_goals[agent_index] = (gx, gy)

        # Add the last state
        if timestep is not None and current_map and starts and current_goals:
            dynamic_states.append((timestep, current_map, starts, current_goals))

    return dynamic_states


if __name__ == '__main__':
    # Set up argument parsing
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, required=True,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + SOLVER)

    # Parse arguments
    args = parser.parse_args()

    # Load the dynamic states from the specified input file
    dynamic_states = import_dynamic_mapf_instance(args.instance)

    # Debugging: Print parsed dynamic states
    for state in dynamic_states:
        print(f"Timestep: {state[0]}")
        print("Map:")
        for row in state[1]:
            print("".join('@' if cell else '.' for cell in row))
        print(f"Starts: {state[2]}")
        print(f"Goals: {state[3]}")
        print()

    paths = None
    current_timestep = 0

    # Process each dynamic state
    for timestep, my_map, starts, goals in dynamic_states:
        print(f"*** Timestep {timestep} ***")

        # Wait until the current timestep
        while current_timestep < timestep:
            current_timestep += 1

        # Validate positions before solving
        validate_positions(my_map, starts, goals)

        # Solve the current state using the selected solver
        if args.solver == "CBS":
            print("***Run CBS***")
            solver = CBSSolver(my_map, starts, goals)
            paths = solver.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals, dynamic_states)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        # Compute and display the cost
        cost = get_sum_of_cost(paths)
        print(f"Cost at Timestep {timestep}: {cost}")
        print(f"Paths at timestep {timestep}: {paths}")

        # Visualization
        if not args.batch:
            animation = Animation(dynamic_states, paths)
            animation.show()
