import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class SpaceTimePlanningSolver(object):
    """
        From Individual Assignment
        Cooperative agent planning upgraded from spatial A*: Space-Time A*.
        https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf

    """

    def __init__(self, my_map, starts, goals, constraints, max_steps):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.constraints = constraints
        self.max_steps = max_steps

        self.CPU_time = 0
        self.num_of_expanded = 0
        self.num_of_generated = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        constraints = self.constraints
        result = [None] * self.num_of_agents # all result paths of all agents

        # Space-Time reservation table using dictionary.
        # key: timestep
        # value: dict(): key = agent, value = location
        reservation_table = dict()

        for i in range(self.num_of_agents):
            path, expansions, generated = a_star(self.my_map, self.starts[i], self.goals[i],
                          self.heuristics[i], i, constraints)
            if path is None:
                return []

            # Accumulate statistics
            self.num_of_expanded += expansions
            self.num_of_generated += generated

            result[i] = path

            # update reservation table for every agent path solution found
            for timestep,p in enumerate(path):
                if timestep not in reservation_table:
                    reservation_table[timestep] = dict()
                if i not in reservation_table[timestep]:
                    reservation_table[timestep][i] = []
                reservation_table[timestep][i].append(p)
                if timestep > 0:
                    if timestep - 1 not in reservation_table:
                        reservation_table[timestep-1] = dict()
                    if i not in reservation_table[timestep]:
                        reservation_table[timestep][i] = []
                    reservation_table[timestep-1][i].append(p)

            # add constraints for the next agent
            if i < self.num_of_agents - 1:
                agent = i+1
                for timestep, reserve_list in reservation_table.items():
                    for reserve_agent, loc_list in reserve_list.items():
                        if timestep < len(path) - 1: # add vertex constraint exclude last pos of path
                            constraints.append({'agent': agent, 'loc': [loc_list[0]], 'timestep': timestep,
                                                'type': 'vertex'})
                        if len(loc_list) > 1: # add edge constraints exclude first timestep in reserve table
                            constraints.append({'agent': agent, 'loc': [loc_list[1], loc_list[0]], 'timestep':
                                                    timestep+1, 'type': 'edge'})
                constraints.append({'agent': agent, 'loc': [path[-1]], 'timestep': len(path) - 1,
                                                                'type': 'inf'})

        self.CPU_time = timer.time() - start_time

        # Print results including expansions and generated
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(f"Expanded nodes: {self.num_of_expanded}")
        print(f"Generated nodes: {self.num_of_generated}")
        return result
