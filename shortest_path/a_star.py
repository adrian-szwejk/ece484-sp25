import heapq
def best_first_search(starting_state):
    '''
    Implementation of best A* algorithm

    Input:
        starting_state: an AbstractState object

    Return:
        A path consisting of a list of AbstractState states
        The first state should be starting_state
        The last state should have state.is_goal() == True
    '''
    visited_states = {starting_state: (None, 0)}

    frontier = []
    heapq.heappush(frontier, starting_state)
    
    while frontier:
        current = heapq.heappop(frontier)
        if current.is_goal():
            return backtrack(visited_states, current)
        # Expand neighbors
        neighbors = current.get_neighbors()
        for neighbor in neighbors:
            # Compute the distance to this neighbor
            new_dist = neighbor.dist_from_start + neighbor.h
            if neighbor not in visited_states.keys():
                visited_states[neighbor] = (current, new_dist)
                heapq.heappush(frontier, neighbor)
            elif visited_states[neighbor][1] > new_dist:
                visited_states[neighbor] = (current, new_dist)
    return []
def backtrack(visited_states, goal_state):
    path = []
    current = goal_state
    while current is not None:
        path.append(current)
        current = visited_states[current][0]
    return path[::-1]