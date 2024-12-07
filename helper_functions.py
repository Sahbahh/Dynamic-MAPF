from single_agent_planner import a_star

"""
Obstacle dictionary:
 - key, value = timestep, obstacle information: {appearance_timestep, obstacle_location}
 - Appearance timestep is how many timesteps this obstacle will appear for. By precalculating this
   we can add constraints to agents in specific timesteps to avoid said obstacle.
 
 compile_obstacle_dict function:
 
 Inputs:
 - JSON obstacle data
 - timesteps in which obstacles appear
 - number of rows of map
 - number of cols of map
 - max_steps (used to set appearance time of obstacle in case obstacle stay indefinitely without a remove)

 Outputs:
 - dictionary of obstacles: used for adding constraints to agents to make agents avoid obstacles
 
"""
def compile_obstacle_dict(input_data, input_timesteps, rows, cols, max_steps):
    result = dict()
    memo = [[0 for _ in range(cols)] for _ in range(rows)]
    max_obstacle_time = max(input_timesteps)

    for obstacle_time in input_timesteps:
        if str(obstacle_time) in input_data:
            changes = input_data[str(obstacle_time)]

            #only update obstacle dictionary if the timestep has the keyword add or remove obstacles
            for obstacle_location in changes.get("add_obstacles", []):
                memo[obstacle_location[0]][obstacle_location[1]] = obstacle_time

            for obstacle_location in changes.get("remove_obstacles", []):
                # memo saves time most recent obstacle arrive at cell
                appear_time = memo[obstacle_location[0]][obstacle_location[1]]
                appearance_timestep = obstacle_time - appear_time #obstacle appear for how many time steps
                new_c = {'appearance_timestep': appearance_timestep,
                         'loc': (obstacle_location[0],obstacle_location[1])}
                if appear_time not in result:
                    result[appear_time] = []
                result.get(appear_time).append(new_c)
                memo[obstacle_location[0]][obstacle_location[1]] = 0 # reset time of removed obstacle

    # check for obstacles that stay until program ends
    for r in range(len(memo)):
        for c in range(len(memo[0])):
            appear_time = memo[r][c]
            if 0 < appear_time <= max_obstacle_time:
                appearance_timestep = max_steps - appear_time  # no remove command, so obstacle stay indefinitely until
                                                            # max steps
                new_c = {'appearance_timestep': appearance_timestep,
                         'loc': (r,c)}
                if appear_time not in result:
                    result[appear_time] = []
                result.get(appear_time).append(new_c)

    return result

def compile_goals_dict(input_data, input_timesteps):
    result = dict()

    for goal_time in input_timesteps:
        if str(goal_time) in input_data:
            changes = input_data[str(goal_time)]

            # only update goal dictionary if the timestep has the keyword change goals
            for agent, goal_location in enumerate(changes.get("change_goals", [])):
                if goal_time not in result:
                    result[goal_time] = []
                result[goal_time].append({'agent': agent,'loc': goal_location})

    return result

def update_constraints(goal_timestep, paths, goal_list, starts, goals, constraints):
    print("UPDATE CONSTRAINTS")
    print("GOAL TIMESTEP: ",goal_timestep)
    for agent, goal in enumerate(goal_list):
        loc = tuple(goal['loc'])
        goals[agent] = loc

    #current agent position is the new starting point
    for agent in range(0, len(starts)):
        if len(paths[agent]) <= goal_timestep:
            new_start = paths[agent][-1]
        else:
            new_start = paths[agent][goal_timestep]
        starts[agent] = new_start

    # new timestep equals current timestep minus current goal timestep
    # remove constraints with timestep < 0
    memo = []
    for i, constraint in enumerate(constraints):
        new_time = constraint['timestep'] - goal_timestep
        if new_time >= 0:
            constraint['timestep'] = new_time
        else:
            memo.append(i)

    memo.reverse()
    for i in memo:
        constraints.pop(i)


"""
Replan all agent paths every time we update the constraints.
Constraints added may be due to new obstacles, or agent locations.
"""
def replan(number_agents, single_agent_planner_map, starts, goals, heuristics, constraints):
    result = []
    for i in range(number_agents):  # Find path for each agent
        path = a_star(single_agent_planner_map, starts[i], goals[i], heuristics[i],
                      i, constraints)
        if path is None:
            raise BaseException('No solutions')
        result.append(path)

    print("paths:")
    for p in result:
        print(p)
    return result

"""
Change format of the map to use with single_agent_planner.py
"""
def initialize_single_agent_planner_map(map):
    result = []
    for i in range(len(map)):
        new_row = []
        for j in range(len(map[0])):
            if map[i][j] == 0:
                new_row.append(False) # no obstacle
            elif map[i][j] == 1:
                new_row.append(True) # there is an obstacle/wall
        result.append(new_row)
    return result


