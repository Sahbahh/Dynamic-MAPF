import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    for t_step in range(max(len(path1), len(path2))):
        # Check vertex collision
        loc1 = get_location(path1, t_step)
        loc2 = get_location(path2, t_step)
        if loc1 == loc2:
            return {'loc': [loc1], 'timestep': t_step}
        
        # Check edge collision
        if t_step < min(len(path1), len(path2)) - 1:
            next_loc1 = get_location(path1, t_step + 1)
            next_loc2 = get_location(path2, t_step + 1)
            if loc1 == next_loc2 and loc2 == next_loc1:
                return {'loc': [loc1, loc2], 'timestep': t_step + 1}
    return None


def detect_collisions(paths):
    collisions = []
    for agent1 in range(len(paths)):
        for agent2 in range(agent1 + 1, len(paths)):
            collision = detect_collision(paths[agent1], paths[agent2])
            if collision:
                collisions.append({'a1': agent1, 'a2': agent2, 'loc': collision['loc'], 'timestep': collision['timestep']})
    return collisions


def standard_splitting(collision):
    constraints = []
    if len(collision['loc']) == 1:  # Vertex collision
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
    elif len(collision['loc']) == 2:  # Edge collision
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': False})
    return constraints


def disjoint_splitting(collision):
    chosen_agent = random.choice([collision['a1'], collision['a2']])
    other_agent = collision['a1'] if chosen_agent == collision['a2'] else collision['a2']
    constraints = []
    if len(collision['loc']) == 1:  # Vertex collision
        constraints.append({'agent': chosen_agent, 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True})
        constraints.append({'agent': other_agent, 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
    elif len(collision['loc']) == 2:  # Edge collision
        constraints.append({'agent': chosen_agent, 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True})
        constraints.append({'agent': other_agent, 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': False})
    return constraints


class CBSSolver:
    """Conflict-Based Search (CBS) solver for MAPF."""

    def __init__(self, my_map, starts, goals, agent_constraints):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.agent_constraints = agent_constraints  # Include agent constraints for dynamic obstacles

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.open_list = []

        # Compute heuristics for low-level search
        self.heuristics = [compute_heuristics(my_map, goal) for goal in goals]

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, _, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        self.start_time = timer.time()

        # Generate the root node
        root = {
            'cost': 0,
            'constraints': self.agent_constraints,  # Start with dynamic constraints
            'paths': [],
            'collisions': []
        }

        # Find initial path for each agent
        for i in range(self.num_of_agents):
            path, expansions, generated = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
            if path is None:
                raise BaseException("No solutions")
            self.num_of_expanded += expansions
            self.num_of_generated += generated
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # High-level search loop
        while self.open_list:
            node = self.pop_node()

            # If no collisions, return the solution
            if not node['collisions']:
                self.print_results(node)
                return node['paths']

            # Resolve the first collision
            collision = node['collisions'][0]
            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

            for constraint in constraints:
                # agent_id comes from the current constraint
                agent_id = constraint['agent']

                child = {
                    'cost': 0,
                    'constraints': node['constraints'] + [constraint],
                    'paths': list(node['paths']),
                    'collisions': []
                }

                # Replan the path for the affected agent
                path, expansions, generated = a_star(self.my_map,
                                                     self.starts[agent_id],
                                                     self.goals[agent_id],
                                                     self.heuristics[agent_id],
                                                     agent_id,
                                                     child['constraints'])
                if path is None:
                    continue

                self.num_of_expanded += expansions
                self.num_of_generated += generated

                child['paths'][agent_id] = path
                child['cost'] = get_sum_of_cost(child['paths'])
                child['collisions'] = detect_collisions(child['paths'])
                self.push_node(child)

        raise BaseException("No solutions found")

    def print_results(self, node):
        print("\nFound a solution!\n")
        CPU_time = timer.time() - self.start_time
        print(f"CPU time (s): {CPU_time:.2f}")
        print(f"Sum of costs: {get_sum_of_cost(node['paths'])}")
        print(f"Expanded nodes: {self.num_of_expanded}")
        print(f"Generated nodes: {self.num_of_generated}")
