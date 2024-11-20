#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

Colors = ['green', 'blue', 'orange']


class Animation:
    def __init__(self, dynamic_states, paths):
        """
        :param dynamic_states: List of tuples [(timestep, my_map, starts, goals), ...].
        :param paths: List of agent paths.
        """
        self.dynamic_states = dynamic_states
        self.paths = []

        # Initialize map for the first state
        initial_state = dynamic_states[0]
        self.my_map = np.flip(np.transpose(initial_state[1]), 1)
        self.starts = [(s[1], len(self.my_map[0]) - 1 - s[0]) for s in initial_state[2]]
        self.goals = [(g[1], len(self.my_map[0]) - 1 - g[0]) for g in initial_state[3]]

        # Convert paths to visualization-friendly coordinates
        for path in paths:
            self.paths.append([(loc[1], len(self.my_map[0]) - 1 - loc[0]) for loc in path])

        aspect = len(self.my_map) / len(self.my_map[0])  # Calculate aspect ratio

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.ax.set_aspect(aspect)  # Set aspect ratio here
        self.ax.set_xlim(-0.5, len(self.my_map) - 0.5)
        self.ax.set_ylim(-0.5, len(self.my_map[0]) - 0.5)
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        self.original_colors = {}  # New: Dictionary to store original face colors

        # Initialize agents
        for i, start in enumerate(self.starts):
            agent_circle = Circle(start, 0.3, facecolor=Colors[i % len(Colors)], edgecolor='black')
            self.agents[i] = agent_circle
            self.original_colors[i] = Colors[i % len(Colors)]  # Store original color
            self.patches.append(agent_circle)

            agent_label = self.ax.text(start[0], start[1] + 0.5, str(i), ha='center', va='center')
            self.agent_names[i] = agent_label
            self.artists.append(agent_label)

        self.current_timestep = 0
        self.total_timesteps = max([state[0] for state in dynamic_states]) + 1

        # Initialize the animation
        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=self.total_timesteps * 10,
                                                 interval=100,
                                                 blit=True)

        print(f"My map dimensions: {len(self.my_map)} x {len(self.my_map[0])}")
        print(f"Starts: {self.starts}")
        print(f"Goals: {self.goals}")
        print(f"Paths: {self.paths}")

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        # Clear the axes
        self.ax.clear()

        # Set axis limits
        x_min, y_min = -0.5, -0.5
        x_max, y_max = len(self.my_map) - 0.5, len(self.my_map[0]) - 0.5
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)

        # Add the map grid
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j]:  # Blocked cell
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))

        # Add agent starting positions
        for i, start in enumerate(self.starts):
            agent_circle = Circle(start, 0.3, facecolor=Colors[i % len(Colors)], edgecolor='black')
            self.agents[i] = agent_circle
            self.patches.append(agent_circle)

            # Add agent labels
            agent_label = self.ax.text(start[0], start[1] + 0.5, str(i), ha='center', va='center')
            self.agent_names[i] = agent_label
            self.artists.append(agent_label)

        # Add goal markers
        for i, goal in enumerate(self.goals):
            self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5,
                                          facecolor=Colors[i % len(Colors)], edgecolor='black', alpha=0.5))

        # Add patches and artists to the axes
        for patch in self.patches:
            self.ax.add_patch(patch)
        for artist in self.artists:
            self.ax.add_artist(artist)

        # Force canvas redraw
        self.fig.canvas.draw_idle()

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
    
    def update_map(self, timestep):
        for state_timestep, my_map, starts, goals in self.dynamic_states:
            if state_timestep == timestep:
                # Update the map
                self.my_map = np.flip(np.transpose(my_map), 1)
                self.starts = [(s[1], len(self.my_map[0]) - 1 - s[0]) for s in starts]
                self.goals = [(g[1], len(self.my_map[0]) - 1 - g[0]) for g in goals]

                # Clear and redraw patches
                self.ax.clear()
                x_min, y_min = -0.5, -0.5
                x_max, y_max = len(self.my_map) - 0.5, len(self.my_map[0]) - 0.5
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)

                # Redraw map
                self.patches = []
                for i in range(len(self.my_map)):
                    for j in range(len(self.my_map[0])):
                        if self.my_map[i][j]:
                            self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))

                # Redraw goals
                for i, goal in enumerate(self.goals):
                    self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5,
                                                facecolor=Colors[i % len(Colors)], edgecolor='black', alpha=0.5))

                # Reset agents
                for i, agent in enumerate(self.agents.values()):
                    agent.center = self.starts[i]

                # Redraw all patches and artists
                for patch in self.patches:
                    self.ax.add_patch(patch)
                for artist in self.artists:
                    self.ax.add_artist(artist)


    def animate_func(self, t):
        timestep = int(t / 10)

        # Check if we need to update the map
        if timestep != self.current_timestep:
            self.update_map(timestep)
            self.current_timestep = timestep

        # Update agent positions
        for k in range(len(self.paths)):
            pos = self.get_state(t / 10, self.paths[k])
            self.agents[k].center = (pos[0], pos[1])
            self.agent_names[k].set_position((pos[0], pos[1] + 0.5))

        # Reset all colors using the `original_colors` dictionary
        for agent_id, agent in self.agents.items():
            agent.set_facecolor(self.original_colors[agent_id])

        # Check for collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')  # Collision detected
                    d2.set_facecolor('red')
                    print(f"COLLISION! (agent-agent) ({i}, {j}) at time {t / 10}")

        return self.patches + self.artists

