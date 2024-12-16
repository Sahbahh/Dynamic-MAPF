import heapq
import math

"""
From individual assignment.
Defines Space-Time A* search algorithm.
"""

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent, goal_location):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.

    goal_constraints = None

    result = dict()
    inf_constraints = dict()
    for c in constraints:
        if c['agent'] == agent:
            if not 'positive' in c:  # disjoint splitting support
                c['positive'] = False  # for standard splitting, constraint is always negative.
            if 'timestep' not in c:
                raise KeyError(f"Constraint missing 'timestep': {c}")
            timestep = c['timestep']

            if not 'type' in c:
                n_loc = len(c['loc'])
                if n_loc == 1: c['type'] = 'vertex'
                elif n_loc == 2: c['type'] = 'edge'
                else: c['type'] = None

            new_c = {'loc': c['loc'], 'type': c['type'], 'positive': c['positive']}

            if c['type'] in ['vertex', 'edge']:
                if timestep in result:
                    result[timestep].append(new_c)
                else:
                    result[timestep] = [new_c]
            elif c['type'] == 'inf':
                # add special inf constraints
                inf_constraints[c['loc'][0]] = new_c


            # if this is a goal constraint. Only applies to a vertex constraint.
            if c['loc'][0] == goal_location and c['type'] == 'vertex':
                if goal_constraints is None:
                    goal_constraints = c['timestep']
                else:
                    goal_constraints = max(goal_constraints, c['timestep'])


    return result, goal_constraints, inf_constraints

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table, inf_constraints):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # only check constraint if next time is in the constraint table
    if next_time in constraint_table:
        constraints = constraint_table[next_time]
        for c in constraints:
            if [next_loc] == c['loc'] or [curr_loc, next_loc] == c['loc']:
                return True
    # check if next location is at an infinite constraint then return True,
    # child node will not be added to closed list.
    for inf_c, _ in inf_constraints.items():
        if inf_c == next_loc:
            return True
    return False



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position (tuple)
        goal_loc    - goal position (tuple)
        agent       - the agent that is being re-planned
        constraints - constraints defining where the agent should or cannot go at each timestep
    """
    # Track expansions and generated counts
    expansions = 0
    generated = 0

    # precalculate max steps
    max_steps = sum(row.count(False) for row in my_map)
    max_steps += math.ceil(len(constraints)/2)

    def time_limit_check(node):
        return node['g_val'] > max_steps

    agent_constraints, goal_constraint, inf_constraints = build_constraint_table(constraints, agent, goal_loc)

    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None}
    push_node(open_list, root)
    generated += 1
    closed_list[(root['loc'],0)] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        expansions += 1  # We just expanded a node by popping it from the open_list

        # Check goal condition
        if curr['loc'] == goal_loc:
            if goal_constraint is None or curr['g_val'] > goal_constraint:
                return get_path(curr), expansions, generated

        if time_limit_check(curr):
            return None, expansions, generated

        next_timestep = curr['g_val'] + 1

        for direction in range(5): # four directions + 1 waiting direction
            child_loc = move(curr['loc'], direction)

            # Check if out of map or blocked
            if(child_loc[0] < 0 or child_loc[0] >= len(my_map)
                or child_loc[1] < 0 or child_loc[1] >= len(my_map[0])):
                continue

            if my_map[child_loc[0]][child_loc[1]]:
                continue

            child = {'loc': child_loc,
                     'g_val': next_timestep,
                     'h_val': h_values[child_loc],
                     'parent': curr}

            # Check constraints
            if is_constrained(curr['loc'], child_loc, next_timestep, agent_constraints, inf_constraints):
                continue

            if (child['loc'], next_timestep) in closed_list:
                existing_node = closed_list[(child['loc'],next_timestep)]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], next_timestep)] = child
                    push_node(open_list, child)
                    generated += 1
            else:
                closed_list[(child['loc'], next_timestep)] = child
                push_node(open_list, child)
                generated += 1

    # No solution found
    return None, expansions, generated
