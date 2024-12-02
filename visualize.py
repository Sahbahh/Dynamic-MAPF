#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

# Define colors for agents
Colors = ['green', 'blue', 'orange', 'red', 'purple', 'yellow']

class Animation:
    def __init__(self, dynamic_states, paths):
        """
        :param dynamic_states: List of tuples [(timestep, my_map, starts, goals), ...].
        :param paths: List of agent paths.
        """
        self.dynamic_states = dynamic_states
        self.paths = paths

        # Initialize the first map state
        initial_state = dynamic_states[0]
        self.my_map = initial_state[1]
        self.starts = initial_state[2]
        self.goals = initial_state[3]

        # Set up figure and axes
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-0.5, len(self.my_map[0]) - 0.5)
        self.ax.set_ylim(-0.5, len(self.my_map) - 0.5)
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        # Initialize lists for map elements
        self.map_patches = []  # Obstacles and goals
        self.agent_patches = {}  # Agents (circles)
        self.agent_labels = {}  # Agent numbers

        # Draw the initial map
        self._draw_map()

        # Initialize agents
        for i, start in enumerate(self.starts):
            agent_circle = Circle((start[1], len(self.my_map) - 1 - start[0]), 0.3, facecolor=Colors[i % len(Colors)], edgecolor='black')
            self.agent_patches[i] = agent_circle
            self.ax.add_patch(agent_circle)

            agent_label = self.ax.text(start[1], len(self.my_map) - 1 - start[0] + 0.4, str(i), color='black', ha='center', va='center')
            self.agent_labels[i] = agent_label

        # Initialize animation parameters
        self.current_timestep = 0
        self.total_timesteps = max([state[0] for state in dynamic_states]) + 10

        # Create animation
        self.animation = animation.FuncAnimation(self.fig, self._animate_frame, init_func=self._init_animation,
                                                 frames=self.total_timesteps * 10, interval=100, blit=False)

    def _draw_map(self):
        """Draw obstacles and goals on the map."""
        # Clear existing patches
        for patch in self.map_patches:
            patch.remove()
        self.map_patches = []

        # Draw obstacles
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j]:  # Blocked cell
                    rect = Rectangle((j - 0.5, len(self.my_map) - 1 - i - 0.5), 1, 1, facecolor='gray',
                                     edgecolor='black')
                    self.map_patches.append(rect)
                    self.ax.add_patch(rect)

        # Draw goals
        # Draw goals
        for i, goal in enumerate(self.goals):
            if goal:  # Skip None goals
                goal_marker = Rectangle((goal[1] - 0.25, len(self.my_map) - 1 - goal[0] - 0.25), 0.5, 0.5,
                                        facecolor=Colors[i % len(Colors)], edgecolor='black', alpha=0.5)
                self.map_patches.append(goal_marker)
                self.ax.add_patch(goal_marker)

    def _init_animation(self):
        """Initialize the animation."""
        self._draw_map()
        return list(self.map_patches) + list(self.agent_patches.values()) + list(self.agent_labels.values())

    def _animate_frame(self, t):
        """Animate each frame."""
        timestep = int(t / 10)

        # Ensure the animation continues for at least 10 timesteps after the last update
        if timestep > self.total_timesteps:
            return []  # Stop the animation after buffer period

        # Update the map if needed
        if timestep != self.current_timestep:
            print(f"Updating map at timestep {timestep}")
            self._update_map(timestep)
            self.current_timestep = timestep

        # Update agent positions
        for i, path in enumerate(self.paths):
            if len(path) > 0:
                pos = self._get_interpolated_position(t / 10, path)
                self.agent_patches[i].center = (pos[1], len(self.my_map) - 1 - pos[0])  # Update circle position
                self.agent_labels[i].set_position(
                    (pos[1], len(self.my_map) - 1 - pos[0] + 0.4))  # Update label position

        # Reset agent colors
        for agent_id, agent_circle in self.agent_patches.items():
            agent_circle.set_facecolor(Colors[agent_id % len(Colors)])

        # Check for collisions
        self._check_collisions()

        return list(self.map_patches) + list(self.agent_patches.values()) + list(self.agent_labels.values())

    def _update_map(self, timestep):
        """Update the map and goals for a new timestep."""
        updated = False
        for state_timestep, my_map, starts, goals in self.dynamic_states:
            if state_timestep == timestep:
                self.my_map = my_map
                self.starts = starts
                self.goals = goals  # No additional transformation here
                self._draw_map()
                print(f"Map and goals updated at timestep {timestep}.")
                updated = True
                break
        if not updated:
            print(f"No updates found for timestep {timestep}.")

    @staticmethod
    def _get_interpolated_position(t, path):
        """Interpolate agent position between timesteps."""
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            return pos_last + (pos_next - pos_last) * (t - int(t))

    def _check_collisions(self):
        """Check for agent collisions."""
        agent_positions = [np.array(agent.center) for agent in self.agent_patches.values()]
        for i in range(len(agent_positions)):
            for j in range(i + 1, len(agent_positions)):
                if np.linalg.norm(agent_positions[i] - agent_positions[j]) < 0.7:
                    self.agent_patches[i].set_facecolor('red')
                    self.agent_patches[j].set_facecolor('red')
                    print(f"COLLISION! Agents {i} and {j} collided at timestep {self.current_timestep}")

    def show(self):
        """Show the animation."""
        plt.show()
