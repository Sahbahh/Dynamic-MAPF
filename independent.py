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
        self.goals = goals
        self.dynamic_states = dynamic_states
        self.num_of_agents = len(goals)
        self.paths = [[] for _ in range(self.num_of_agents)]  # Store paths for agents
        self.CPU_time = 0

        # Compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def update_map(self, timestep):
        """Update the map, start positions, and goals at the given timestep."""
        for state_timestep, my_map, starts, goals in self.dynamic_states:
            if state_timestep == timestep:
                # Update map, starts, and goals dynamically
                self.my_map = my_map
                self.starts = starts
                self.goals = goals

                # Recompute heuristics for the new goals
                self.heuristics = []
                for goal in self.goals:
                    self.heuristics.append(compute_heuristics(self.my_map, goal))

                print(f"Dynamic map updated at timestep {timestep}.")
                return True
        return False

    def find_solution(self):
        """Find paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        timestep = 0
        max_timesteps = max(state[0] for state in self.dynamic_states)

        while True:
            # Update the map if a dynamic state exists at the current timestep
            self.update_map(timestep)

            # Check and calculate paths for all agents
            all_agents_at_goals = True
            for i in range(self.num_of_agents):
                # If agent already has a valid path to its goal, skip recalculating
                if self.paths[i] and self.paths[i][-1] == self.goals[i]:
                    continue

                # Otherwise, attempt to calculate a new path
                print(f"Calculating path for agent {i} at timestep {timestep}...")
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, [])
                if path is None:
                    print(f"No path found for agent {i} at timestep {timestep}.")
                    raise BaseException(f"Agent {i} cannot reach its goal.")
                self.paths[i] = path
                all_agents_at_goals = False  # Not all agents are at their goals

            # Check if all agents have reached their goals
            if all_agents_at_goals:
                print("All agents have successfully reached their goals!")
                break

            # Increment timestep
            timestep += 1
            if timestep > max_timesteps * 2:  # Safety limit to prevent infinite loops
                print("Maximum timesteps exceeded. Terminating...")
                raise BaseException("Agents could not reach goals within a reasonable time.")

        self.CPU_time = timer.time() - start_time

        print("\nFound a solution!")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(self.paths)))
        print(f"Paths: {self.paths}")

        return self.paths
