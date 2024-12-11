import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals, constraints):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.constraints = constraints

        self.CPU_time = 0
        self.num_of_expanded = 0
        self.num_of_generated = 0

        # Compute heuristics for each agent's goal
        self.heuristics = [compute_heuristics(my_map, goal) for goal in self.goals]

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        start_time = timer.time()
        constraints = self.constraints
        priorities = list(range(self.num_of_agents))  
        result = [None] * self.num_of_agents

        for i in range(self.num_of_agents):
            current_agent = priorities[i]
            # Note: We now capture expansions and generated from a_star
            path, expansions, generated = a_star(
                self.my_map, 
                self.starts[current_agent], 
                self.goals[current_agent],
                self.heuristics[current_agent], 
                current_agent, 
                constraints
            )

            if path is None:
                raise BaseException('No solutions')

            # Accumulate statistics
            self.num_of_expanded += expansions
            self.num_of_generated += generated

            result[current_agent] = path

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            # all constraints of prev agents is also a constraint for the next agent
            #
            for j in range(i+1, self.num_of_agents):
                agent = priorities[j]
                for timestep in range(len(path) - 1):
                    curr_loc = path[timestep]
                    next_loc = path[timestep + 1]
                    constraints.append({'agent': agent, 'loc': [curr_loc], 'timestep': timestep,
                                        'type': 'vertex'})
                    constraints.append({'agent': agent, 'loc': [next_loc,curr_loc], 'timestep': timestep + 1,
                                        'type': 'edge'})
                # apply all existing goals as constraints
                constraints.append({'agent': agent, 'loc': [path[-1]], 'timestep': len(path) - 1,
                                        'type': 'inf'})

        self.CPU_time = timer.time() - start_time

        # Print results including expansions and generated
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(f"Expanded nodes: {self.num_of_expanded}")
        print(f"Generated nodes: {self.num_of_generated}")
        print(result)
        return result
