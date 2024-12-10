from cbs import CBSSolver  # Import your CBS implementation
import random
from cbs import detect_collisions

from single_agent_planner import a_star, compute_heuristics


class LNSSolver:
    def __init__(self, my_map, starts, goals, constraints):
        """
        Initialize Large Neighbourhood Search solver

        :param my_map: 2D grid representing the map (0 is free, 1 is obstacle)
        :param starts: List of start positions for each agent
        :param goals: List of goal positions for each agent
        :param constraints: List of constraints for path planning
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.constraints = constraints

        # Precompute heuristics for each goal
        self.heuristics = [compute_heuristics(my_map, goal) for goal in goals]

    def initialize_solution_with_cbs(self):
        """
        Use CBS to find the initial solution, resolving collisions.
        """
        print("Initializing solution using CBS...")
        cbs_solver = CBSSolver(self.my_map, self.starts, self.goals, self.constraints)
        try:
            solution = cbs_solver.find_solution(disjoint=True)
        except BaseException as e:
            print(f"CBS failed to find a solution: {e}")
            return None
        return solution

    def destroy(self, solution, destroy_percentage=0.3):
        """
        Destroy (remove) a subset of agent paths

        :param solution: Current solution of agent paths
        :param destroy_percentage: Percentage of agents to remove
        :return: Indices of agents to be replanned
        """
        num_agents = len(solution)
        num_destroy = max(1, int(num_agents * destroy_percentage))
        return random.sample(range(num_agents), num_destroy)

    def repair(self, solution, destroy_indices):
        """
        Repair the solution by replanning for destroyed agents

        :param solution: Current solution of agent paths
        :param destroy_indices: Indices of agents to replan
        :return: Updated solution
        """
        repaired_solution = solution.copy()

        for agent in destroy_indices:
            path = a_star(
                self.my_map,
                self.starts[agent],
                self.goals[agent],
                self.heuristics[agent],
                agent,
                self.constraints
            )
            if path is None:
                print(f"Agent {agent} failed to find a path. Keeping its original path.")
                continue

            repaired_solution[agent] = path
        return repaired_solution

    def find_solution(self, max_iterations=100, destroy_percentage=0.3):
        """
        Find a solution using Large Neighbourhood Search

        :param max_iterations: Maximum number of LNS iterations
        :param destroy_percentage: Percentage of agents to destroy in each iteration
        :return: Final solution (list of paths)
        """
        # Initialize solution with CBS
        solution = self.initialize_solution_with_cbs()
        if solution is None:
            return None

        best_solution = solution
        best_cost = self.calculate_solution_cost(solution)

        for iteration in range(max_iterations):
            print(f"--- LNS Iteration {iteration + 1} ---")

            # Detect collisions in the current solution
            collisions = detect_collisions(solution)
            if not collisions:
                print("No collisions detected. Solution found!")
                return solution

            # Destroy phase: Select colliding agents for replanning
            destroy_indices = list(set([c['a1'] for c in collisions] + [c['a2'] for c in collisions]))

            # Repair phase: Find new paths for destroyed agents
            new_solution = self.repair(solution, destroy_indices)

            # Calculate new solution cost
            new_cost = self.calculate_solution_cost(new_solution)

            # Accept the new solution if it's better and has no collisions
            new_collisions = detect_collisions(new_solution)
            if new_cost < best_cost and not new_collisions:
                print("Improved solution found!")
                best_solution = new_solution
                best_cost = new_cost
                solution = new_solution
            else:
                print("No improvement in this iteration.")

        print("LNS failed to find a better solution within the maximum iterations.")
        return best_solution

    def calculate_solution_cost(self, solution):
        """
        Calculate the total cost of the solution

        :param solution: List of paths for all agents
        :return: Total path length cost
        """
        return sum(len(path) - 1 for path in solution)
