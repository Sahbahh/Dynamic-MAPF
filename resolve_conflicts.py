

def resolve_conflicts(name, conflicts, goal, grid):
    """
        Calls the appropriate search algorithm.

        Args:
            name (str): Algorithm name (e.g., "a_star", "cbs").
            conflicts (list of set): Current positions of all agents.
            goal (array): Goal positions for all agents.
            grid (list): The map grid (2D list with 0 for free, 1 for obstacles).

        Returns:
            array: updated positions of all agents such that no agent collide with each other
        """
    updated_positions = []
    for agent1, agent2, conflict_location in conflicts:
        print("AGENT CONFLICT:")
        print("Conflict between agent ", agent1 + 1," and agent ", agent2 + 1, " at ", conflict_location)

    return updated_positions

