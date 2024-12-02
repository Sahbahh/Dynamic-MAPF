# main.py

from dynamic_map import DynamicMap
from input_parser import parse_input
from independent import independent_solver
from map_validator import validate_map  # Import the validator
import matplotlib.pyplot as plt
import json

def main():
    # Set up argument parser
    import argparse
    parser = argparse.ArgumentParser(description="Dynamic Multi-Agent Path Finding (MAPF) Simulation")
    parser.add_argument("input_file", type=str, help="Path to the input JSON file")

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

    # Simulate movement step-by-step
    max_steps = 100  # Maximum allowed steps to prevent infinite loops
    timesteps = sorted(map(int, input_data.keys()))  # Ensure timesteps are processed in order
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

        # Update agent positions using the independent solver
        agents = independent_solver(map_grid.map_grid, agents, agent_goals, dynamic_changes=changes)

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