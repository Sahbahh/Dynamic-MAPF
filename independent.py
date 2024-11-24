import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost

class IndependentSolver(object):
    """A planner that plans for each robot independently."""

    def __init__(self, my_map, starts, goals, dynamic_states):
        """
        my_map         - list of lists specifying obstacle positions
        starts         - [(x1, y1), (x2, y2), ...] list of start locations
        goals          - [(x1, y1), (x2, y2), ...] list of goal locations
        dynamic_states - list of tuples [(timestep, my_map, starts, goals), ...] for dynamic updates
        """
        self.my_map = my_map
        self.starts = starts
        self.original_goals = goals  # Store the original goals
        self.current_goals = list(goals)  # Current goals, updated dynamically
        self.dynamic_states = dynamic_states
        self.num_of_agents = len(goals)
        self.paths = [[] for _ in range(self.num_of_agents)]  # Store paths for agents
        self.CPU_time = 0

        # Compute heuristics for the low-level search
        self.heuristics = [compute_heuristics(my_map, goal) for goal in self.original_goals]

    def update_map(self, timestep):
        """Update the map, start positions, and goals at the given timestep."""
        updated = False
        for state_timestep, my_map, starts, goals in self.dynamic_states:
            if state_timestep == timestep:
                # Update the map, starts, and goals
                self.my_map = my_map
                self.starts = starts
                self.current_goals = goals  # Use the new goals

                # Recompute heuristics for the new goals
                self.heuristics = [compute_heuristics(self.my_map, goal) for goal in self.current_goals]
                print(f"Dynamic state updated at timestep {timestep}: starts={self.starts}, goals={self.current_goals}")

                # Mark as updated
                updated = True
                break

        if not updated:
            print(f"No updates found for timestep {timestep}.")
        return updated

    def find_solution(self):
        """Find paths for all agents from their start locations to their goal locations."""
        start_time = timer.time()
        timestep = 0
        max_timesteps = max(state[0] for state in self.dynamic_states)

        # Initially calculate paths to the original goals
        for i in range(self.num_of_agents):
            print(f"Calculating initial path for agent {i}...")
            path = a_star(self.my_map, self.starts[i], self.original_goals[i], self.heuristics[i], i, [])
            if path is None:
                print(f"No path found for agent {i} to its original goal.")
                raise BaseException(f"Agent {i} cannot reach its goal.")
            self.paths[i] = path

        while True:
            # Update the map and goals dynamically if a new timestep is reached
            if self.update_map(timestep):
                # Replan for agents based on the new goals
                for i in range(self.num_of_agents):
                    # Replan from the agent's current position
                    current_position = self.paths[i][min(timestep, len(self.paths[i]) - 1)]
                    print(
                        f"Replanning path for agent {i} from position {current_position} to new goal {self.current_goals[i]} at timestep {timestep}...")
                    path = a_star(self.my_map, current_position, self.current_goals[i], self.heuristics[i], i, [])
                    if path is None:
                        print(f"No path found for agent {i} to its new goal at timestep {timestep}.")
                        raise BaseException(f"Agent {i} cannot reach its new goal.")
                    self.paths[i] = self.paths[i][:timestep] + path  # Keep the old path up to the current timestep

            # Check if all agents have reached their final goals
            all_agents_at_goals = True
            for i in range(self.num_of_agents):
                if self.paths[i][-1] != self.current_goals[i]:
                    all_agents_at_goals = False
                    break

            if all_agents_at_goals and timestep > max_timesteps:
                print("All agents have successfully reached their goals!")
                break

            # Increment timestep
            timestep += 1
            if timestep > max_timesteps + 20:  # Safety limit to prevent infinite loops
                print("Maximum timesteps exceeded. Terminating...")
                raise BaseException("Agents could not reach goals within a reasonable time.")

        self.CPU_time = timer.time() - start_time

        print("\nFound a solution!")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(self.paths)))
        print(f"Paths: {self.paths}")

        return self.paths
