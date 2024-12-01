# dynamic_map.py

import matplotlib.pyplot as plt
import numpy as np
import random
from matplotlib.colors import ListedColormap


class DynamicMap:
    """
    Manages a dynamic map for MAPF

    The dynamic map includes obstacles, agents, and their goals, with the ability to update and visualize the map dynamically
    as changes occur 

    Attributes:
        rows (int): Number of rows in the map grid.
        cols (int): Number of columns in the map grid.
        map_grid (list): A 2D grid representing the map (0 for free cells, 1 for obstacles).
        agent_goals (list): List of current goal positions for all agents.
        agents (list): List of current positions for all agents.
        agent_colors (list): Colors assigned to agents for visualization purposes.
        fig, ax (matplotlib objects): Used to display and update the map dynamically.
    """

    def __init__(self, rows, cols):
        """
        Initialize the map with a specified number of rows and columns.

        Args:
            rows (int): Number of rows in the map grid.
            cols (int): Number of columns in the map grid.
        """
        self.rows = rows
        self.cols = cols
        self.map_grid = [[0 for _ in range(cols)] for _ in range(rows)]  # 0: Free, 1: Obstacle
        self.agent_goals = []
        self.agents = []
        self.agent_colors = []
        self.fig, self.ax = None, None


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


    def initialize_animation(self):
        """
        Initialize the figure and axis for animating the dynamic map.
        """
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_title("Dynamic Map Animation")


    def apply_changes(self, changes):
        """
        Apply dynamic changes to the map grid, such as adding or removing obstacles.

        Args:
            changes (dict): A dictionary containing changes to apply.
                - add_obstacles: List of positions to set as obstacles.
                - remove_obstacles: List of positions to clear obstacles.

        Returns:
            list: The updated map grid.
        """
        for pos in changes.get("add_obstacles", []):
            self.map_grid[pos[0]][pos[1]] = 1  # Set cell as obstacle
        for pos in changes.get("remove_obstacles", []):
            self.map_grid[pos[0]][pos[1]] = 0  # Set cell as free
        return self.map_grid


    def update_goals(self, agent_goals, changes):
        """
        Update the goals of agents dynamically.

        Args:
            agent_goals (list): Current goal positions of agents.
            changes (dict): A dictionary containing updated goal positions.

        Returns:
            list: Updated agent goal positions.
        """
        for idx, new_goal in enumerate(changes.get("change_goals", [])):
            if idx < len(agent_goals):
                agent_goals[idx] = new_goal
        return agent_goals


    def update_visualization(self):
        """
        Update and redraw the map visualization with the current agent and goal positions.
        """
        self.ax.clear()  # Clear the previous frame
        grid = np.array(self.map_grid)

        # Create a discrete colormap (0: white for free, 1: black for obstacles)
        cmap = ListedColormap(["white", "black"])
        self.ax.imshow(grid, cmap=cmap, origin="upper", interpolation="none")

        # Add gridlines for better visibility
        for x in range(self.cols + 1):
            self.ax.axvline(x - 0.5, color="black", linewidth=0.5, linestyle="-")
        for y in range(self.rows + 1):
            self.ax.axhline(y - 0.5, color="black", linewidth=0.5, linestyle="-")

        # Set grid limits and disable ticks
        self.ax.set_xlim(-0.5, self.cols - 0.5)
        self.ax.set_ylim(self.rows - 0.5, -0.5)
        self.ax.set_xticks(range(self.cols))
        self.ax.set_yticks(range(self.rows))
        self.ax.grid(False)

        # Plot agents (colored circles) and goals (colored stars)
        for i, agent in enumerate(self.agents):
            self.ax.scatter(agent[1], agent[0], c=[self.agent_colors[i]], s=200, label=f"Agent {i + 1}", zorder=3)

        for i, goal in enumerate(self.agent_goals):
            self.ax.scatter(goal[1], goal[0], c=[self.agent_colors[i]], s=200, marker="*", label=f"Goal {i + 1}", zorder=2)

        # Add a legend to identify agents and their goals
        handles, labels = self.ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        self.ax.legend(by_label.values(), by_label.keys(), loc="upper right")

        # Pause briefly to allow visualization updates
        plt.pause(0.5)
