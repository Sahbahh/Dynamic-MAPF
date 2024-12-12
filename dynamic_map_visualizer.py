#!/usr/bin/env python3
import random
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Rectangle, bbox_artist
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib import animation



class Animation:
    """
    References:
    This document is from individual assignment.
    Toggle pause function: https://matplotlib.org/stable/gallery/animation/pause_resume.html

    Click on graph to pause/resume animation

    """
    def __init__(self, my_map, starts, goals, paths, obstacles, goals_dictionary, algorithm_name=None):
        self.my_map = my_map
        self.starts = []
        for start in starts:
            self.starts.append((start[1], start[0]))
        self.goals = []
        for goal in goals:
            self.goals.append((goal[1], goal[0]))
        self.paths = []

        for path in paths:
            self.paths.append([])
            for loc in path:
                self.paths[-1].append((loc[1], loc[0]))


        self.loading_obstacles = []             # for smooth obstacle transition
        self.patches = []                       # for drawing agents and goals
        self.artists = []                       # for agent names
        self.agents = dict()                    # dictionary of circle objects
        self.agent_names = dict()               # dictionary of text objects
        self.agent_goals = dict()               # dictionary of rectangle objects
        self.create_obstacles = dict()          # dictionary of rectangle objects
        self.delete_obstacles = dict()          # dictionary of rectangle objects
        self.goals_dict = goals_dictionary
        self.obstacles_dict = obstacles

        """
                       Initialize the figure and axis for animating the dynamic map.
        """
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.set_figwidth(12)
        
        # Show the name of the algorithm it is running
        # This happens when you run it from run_all_algorithmspy to compare
        if algorithm_name is not None:
            self.ax.set_title(f"Dynamic Map Animation - {algorithm_name}")
        # This happens when you call it from main.py individually
        else:
            self.ax.set_title("Dynamic Map Animation")


        """
                        Create a discrete colormap (0: white for free, 1: black for obstacles)
                        Set up obstacles creation and deletion dictionary for animation.
                        Transform paths to switch between x and y values
        """

        cmap = ListedColormap(["white", "black"])
        self.grid = np.array(my_map)
        self.ax.imshow(self.grid, cmap=cmap, origin="upper", interpolation="none")

        #setting ticks
        x_ticks = [i for i in range(len(self.my_map[0]))]
        y_ticks = [i for i in range(len(self.my_map))]
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)

        # update appearance timestep to be end time, to use as trigger to set obstacle rectangle to invisible
        if self.obstacles_dict:
            for k, obs_list in self.obstacles_dict.items():
                for ob in obs_list:
                    ob['appearance_timestep'] = k + ob['appearance_timestep']

        # Assign random colors to agents
        self.assign_agent_colors(len(goals))

        # create agents:
        self.T = 0 # for calculating number of frames for animation
        # draw goals first
        for i, goal in enumerate(self.goals):
            self.agent_goals[i] = Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=self.agent_colors[i],
                                          edgecolor='black', alpha=0.5)
            self.patches.append(self.agent_goals[i])

        # draw agents and agent names
        for i in range(len(self.paths)):
            name = str(i)
            self.T = max(self.T, len(paths[i]) - 1)

            self.agents[i] = Circle((starts[i][0], starts[i][1]), 0.2, facecolor=self.agent_colors[i],
                                    edgecolor='black')
            self.patches.append(self.agents[i])

            self.agent_names[i] = self.ax.text(starts[i][0], starts[i][1] + 0.35, name)
            self.agent_names[i].set_horizontalalignment('center')
            self.agent_names[i].set_verticalalignment('center')
            self.artists.append(self.agent_names[i])

        # draw obstacles
        if self.obstacles_dict:
            for timestep, obstacle_list in self.obstacles_dict.items():
                # print(timestep, obstacle_list)
                for obstacle in obstacle_list:
                    obs_loc = obstacle['loc']
                    end_time = obstacle['appearance_timestep']
                    new_rect = Rectangle((obs_loc[1] - 0.5,
                                              obs_loc[0] - 0.5), 1, 1, facecolor='black',
                                              edgecolor='black', alpha=0.5, visible=False)
                    # objects to be created
                    if timestep not in self.create_obstacles:
                        self.create_obstacles[timestep] = []
                    self.create_obstacles[timestep].append(new_rect)

                    # objects to be deleted
                    if end_time not in self.delete_obstacles:
                        self.delete_obstacles[end_time] = []
                    self.delete_obstacles[end_time].append(new_rect)

                    # add rectangle to patches to render
                    self.patches.append(new_rect)

        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 1) * 10 + 5, # add small delay between repeats
                                                 interval=100,
                                                 blit=True, repeat=True, repeat_delay=1000)
        # toggle pause with button event
        self.paused = False
        self.fig.canvas.mpl_connect('button_press_event', self.toggle_pause)

    @staticmethod
    def show():
        plt.show()

    def toggle_pause(self, *args, **kwargs):
        if self.paused:
            self.animation.resume()
        else:
            self.animation.pause()
        self.paused = not self.paused

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)

        # Add gridlines for better visibility
        for x in range(len(self.my_map[0]) + 1):
            self.ax.axvline(x - 0.5, color="black", linewidth=0.5, linestyle="-")
        for y in range(len(self.my_map) + 1):
            self.ax.axhline(y - 0.5, color="black", linewidth=0.5, linestyle="-")

        # Legend
        list_circle_handles = []
        for i, _ in self.agents.items():
            name = 'Agent ' + str(i + 1)
            list_circle_handles.append(mpatches.Patch(color=self.agent_colors[i], label=name))

        plt.legend(handles=list_circle_handles, bbox_to_anchor=(1, 1))

        # reset obstacle locations
        for i, obs_list in self.create_obstacles.items():
            for ob in obs_list:
                if i == 0:
                    ob.set_visible(True)
                else:
                    ob.set_visible(False)

        # reset goal locations
        for i in self.agent_goals:
            self.agent_goals[i].set_x(self.goals[i][0] - 0.25)
            self.agent_goals[i].set_y(self.goals[i][1] - 0.25)

        return self.patches + self.artists


    def animate_func(self, t):
        for k in range(len(self.paths)):
            pos = self.get_state(t / 10, self.paths[k])
            self.agents[k].center = (pos[0], pos[1])
            self.agent_names[k].set_position((pos[0], pos[1] + 0.35))

        # if there are obstacles loading, increment alpha value to create smooth transition
        if len(self.loading_obstacles) != 0:
            complete = []
            for i, ob in enumerate(self.loading_obstacles):
                new_alpha_val = ob.get_alpha() + 0.1
                ob.set_alpha(new_alpha_val)
                if new_alpha_val >= 0.7:
                    complete.append(i)
            complete.reverse()
            for i in complete:
                self.loading_obstacles.pop(i)

        # create obstacles
        if t/10 in self.create_obstacles:
            obs_list = self.create_obstacles.get(t / 10)
            for ob in obs_list:
                ob.set_visible(True)
                ob.set_alpha(0.1)
                self.loading_obstacles.append(ob)

        # delete obstacles
        if t/10 in self.delete_obstacles:
            obs_list = self.delete_obstacles.get(t / 10)
            for ob in obs_list:
                ob.set_visible(False)

        #change position of goals
        if t/10 in self.goals_dict:
            goals_list = self.goals_dict.get(t / 10)
            for goal in goals_list:
                agent = goal['agent']
                new_loc = goal['loc']
                self.agent_goals[agent].set_x(new_loc[1] - 0.25)
                self.agent_goals[agent].set_y(new_loc[0] - 0.25)

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos

    def assign_agent_colors(self, num_agents):
        """
        Assign unique random colors to each agent and their respective goals for visualization.

        Args:
            num_agents (int): Number of agents to assign colors to.
        """
        self.agent_colors = [
            (random.random(), random.random(), random.random())
            for _ in range(num_agents)
        ]