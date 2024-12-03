from dynamic_map import DynamicMap
from input_parser import parse_input
from independent import independent_solver, initialize_single_agent_planner_map, replan, \
    compile_obstacle_dict
from map_validator import validate_map  # Import the validator
import matplotlib.pyplot as plt
import json
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from prioritized import PrioritizedPlanningSolver

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
    agents = [tuple(agent["start"]) for agent in agents_data]
    agent_goals = [tuple(agent["goal"]) for agent in agents_data]

    # Create the map
    map_grid = DynamicMap(rows, cols)

    # Assign random colors to agents
    map_grid.assign_agent_colors(len(agents))

    # Initialize the visualization
    map_grid.initialize_animation()

    ##############################

    number_agents = len(agents)
    single_agent_planner_map = initialize_single_agent_planner_map(map_grid.map_grid)

    heuristics = []
    for goal in agent_goals:
        heuristics.append(compute_heuristics(single_agent_planner_map, goal))


    ###############################
    ## INDEPENDENT AGENT PLANNER ##
    """
    Agent paths are planned independent regardless of obstacles or collisions.
    """
    print("starts: ", agents)
    print("goals: ", agent_goals)

    constraints = []
    result = replan(number_agents, single_agent_planner_map, agents, agent_goals, heuristics, constraints)

    #############################################################

    #### OBSTACLES DETECTION AND AVOIDANCE USING CONSTRAINTS ####
    """
    With this feature, agents will avoid obstacles with added restraints. Agent collision can still happen.
    """

    # time step of obstacles
    obstacle_timesteps = sorted(map(int, input_data.keys()))  # Ensure timesteps are processed in order


    max_steps = 10  # Maximum allowed steps to prevent infinite loops

    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    obstacle_dictionary = compile_obstacle_dict(input_data, obstacle_timesteps, rows, cols, max_steps)


    for timestep, obstacle_list in obstacle_dictionary.items():
        print("timestep: ", timestep, "obstacles: ", obstacle_list)

    for timestep, obstacle_list in obstacle_dictionary.items():
        for obstacle in obstacle_list:
            obstacle_location = obstacle['loc']
            current_time = timestep
            for _ in range(obstacle['appearance_timestep']):
                for agent in range(number_agents):
                    #vertex constraints
                    constraints.append({'agent': agent, 'loc': obstacle_location,
                                        'timestep': current_time, 'type': 'vertex'})

                    #edge constraints
                    start_pos = []
                    for dir in directions:
                        pos = (obstacle_location[0] + dir[0], obstacle_location[1] + dir[1])
                        if pos[0] < 0 or pos[0] >= rows or pos[1] < 0 or pos[1] >= cols: #out of map
                            continue
                        start_pos.append(pos)
                    for pos in start_pos:
                        constraints.append({'agent': agent, 'loc': [pos,obstacle_location],
                                            'timestep': current_time,'type': 'edge'})

                current_time += 1

    # Replan all agent paths using added constraints with a star.
    result = replan(number_agents, single_agent_planner_map, agents, agent_goals, heuristics, constraints)

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
    if args.algorithm == "Prioritized":
        print("***Run Prioritized***")
        solver = PrioritizedPlanningSolver(single_agent_planner_map, agents, agent_goals, constraints)
        result = solver.find_solution()

    #############################################################

    ################ VISUALIZE AGENT PATHS ######################

    # Simulate movement step-by-step
    for step in range(max_steps):
        print(f"Step {step}")

        # Apply changes for the current timestep
        if str(step) in input_data:  # Check if there are changes for this timestep
            changes = input_data[str(step)]
            print(f"Applying changes at timestep {step}: {changes}")
            map_grid.apply_changes(changes)

            # Handle change_goals dynamically
            if "change_goals" in changes:
                for idx, new_goal in enumerate(changes["change_goals"]):
                    if idx < len(agent_goals):  # Ensure the index is valid
                        print(f"Agent {idx + 1} goal changed from {agent_goals[idx]} to {new_goal}")
                        agent_goals[idx] = tuple(new_goal)
        else:
            changes = {}

        # Update agent positions using the independent solver with the selected algorithm
        # agents = independent_solver(map_grid.map_grid, agents, agent_goals, algorithm=args.algorithm, dynamic_changes=changes)

        agents = []
        for path in result:
            if step < len(path):
                agents.append(path[step])
            else:
                agents.append(path[-1])

        # Update the agent positions and visualize the current state
        map_grid.agents = agents  # Set current agents in the map object
        map_grid.agent_goals = agent_goals  # Set current goals
        map_grid.update_visualization()

        # Check if all agents have reached their goals
        if all(agent == goal for agent, goal in zip(agents, agent_goals)):
            print(f"All agents reached their goals in {step} steps.")
            break

    else:
        print("Agents could not reach their goals within the step limit.")

    # Keep the final visualization open
    plt.show()

if __name__ == "__main__":
    main()
