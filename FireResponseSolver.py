import heapq
import time
import math
import itertools

class FireResponseSolver:
    def __init__(self, road_network, time_on_scene, node_locations,
                 emergency_sites, start_station, event_labels=None):
        # Graph representation: node -> list of (neighbor, travel_time)
        self.road_network = road_network
        # Time required to handle emergencies at each location
        self.time_on_scene = time_on_scene
        # Coordinates for heuristic calculation
        self.node_locations = node_locations
        # Set of all emergency sites that must be handled
        self.emergency_sites = set(emergency_sites)
        # Labels for showing us the path we end up taking(not required, but I added it to help presenting )
        self.event_labels = event_labels or {}
        # Initializing the station where we start from
        self.start_station = start_station
        # Initial state: at start station, no emergencies handled yet
        self.start_state = (start_station, frozenset())

    def is_goal(self, state): # Goal state: all emergencies have been handled
        _, handled = state
        return handled == self.emergency_sites

    def get_neighbors(self, state):
        current_loc, handled_set = state
        neighbors = []

        # Move to all connected locations
        for nbr, travel_time in self.road_network.get(current_loc, []):
            neighbors.append(((nbr, handled_set), travel_time))

        # Handle emergency at current location if it hasn't been handled yet
        if current_loc in self.emergency_sites and current_loc not in handled_set:
            new_handled = handled_set.union({current_loc})
            neighbors.append(((current_loc, new_handled), self.time_on_scene[current_loc]))

        return neighbors

    def heuristic_a_star(self, state):
        """
        A* heuristic: estimate time remaining as the sum of:
         - straight-line distance to the nearest unhandled emergency
         - time to handle that emergency
         It underestimates total remaining cost so it won't effect the choises negatively.
        """
        current_loc, handled = state
        remaining = self.emergency_sites - handled
        if not remaining or current_loc not in self.node_locations:
            return 0.0

        current_pos = self.node_locations[current_loc]
        best = float('inf')
        for loc in remaining:
            if loc in self.node_locations:
                pos = self.node_locations[loc]
                dist = math.hypot((current_pos[0] - pos[0]), (current_pos[1] - pos[1]))
                best = min(best, dist + self.time_on_scene.get(loc, 0))
        if best != float('inf'):
            return best
        else:
            return 0.0



def solve(problem, algorithm='ucs', heuristic=None): # For simplicity to compare have one function that tries both methods
    #default solving algorithm is ucs unless we define it

    """
    Generic search solver supporting UCS and A*.
    Uses a priority queue with g-cost + optional heuristic.
    """
    start_time = time.perf_counter()
    start_state = problem.start_state
    counter = itertools.count()  # Tie-breaker for states with same priority
    pq = []

    # Priority: g-cost + h-cost if using A*
    start_priority = 0 + (heuristic(start_state) if algorithm == 'a_star' and heuristic else 0)
    heapq.heappush(pq, (start_priority, next(counter), start_state))

    g_costs = {start_state: 0.0}  # Known cost to reach each state
    parent_map = {start_state: None}  # For reconstructing the path
    nodes_visited = 0

    while pq:
        priority, _, current_state = heapq.heappop(pq)
        nodes_visited += 1
        current_g = g_costs.get(current_state, float('inf'))
        expected_priority = current_g + (heuristic(current_state) if algorithm == 'a_star' and heuristic else 0)

        # Skip outdated entries in the priority queue
        if priority > expected_priority :
            continue

        # Check if current state is the goal
        if problem.is_goal(current_state):
            end_time = time.perf_counter()
            # Reconstruct the path from goal to start
            path = []
            s = current_state
            while s is not None:
                path.append(s)
                s = parent_map.get(s)
            path.reverse()
            return {"path": path, "cost": g_costs[current_state],
                    "nodes_visited": nodes_visited, "time_ms": (end_time - start_time) * 1000.0}

        # Expand neighbors (moves and handling actions)
        for neighbor_state, step_cost in problem.get_neighbors(current_state):
            new_g = current_g + step_cost
            # Record neighbor if it's a new state or if we found a cheaper path
            if neighbor_state not in g_costs or new_g < g_costs[neighbor_state] - 1e-9:
                g_costs[neighbor_state] = new_g
                parent_map[neighbor_state] = current_state
                prio = new_g + (heuristic(neighbor_state) if algorithm == 'a_star' and heuristic else 0)
                heapq.heappush(pq, (prio, next(counter), neighbor_state))

    # No solution found
    end_time = time.perf_counter()
    return {"path": None, "cost": float('inf'),
            "nodes_visited": nodes_visited, "time_ms": (end_time - start_time) * 1000.0}


if __name__ == "__main__":
    # Define road network
    road_network_tirana = {
        'Stacioni Qendror': [('Sheshi Skënderbej', 10), ('Zogu i Zi', 10), ('Komuna e Parisit', 15)],
        'Zogu i Zi': [('Stacioni Qendror', 10), ('Komuna e Parisit', 10)],
        'Sheshi Skënderbej': [('Stacioni Qendror', 10), ('Pazari i Ri', 5), ('Blloku', 10)],
        'Pazari i Ri': [('Sheshi Skënderbej', 5), ('Blloku', 8)],
        'Komuna e Parisit': [('Stacioni Qendror', 15), ('Zogu i Zi', 10), ('Blloku', 10), ('Liqeni', 15)],
        'Blloku': [('Sheshi Skënderbej', 10), ('Komuna e Parisit', 10), ('Pazari i Ri', 8), ('Liqeni', 5)],
        'Liqeni': [('Blloku', 5), ('Komuna e Parisit', 15)]
    }

    # Coordinates for heuristic distance calculation
    locations_tirana = {
        'Stacioni Qendror': (-10, 5),
        'Zogu i Zi': (-15, 10),
        'Sheshi Skënderbej': (0, 0),
        'Pazari i Ri': (5, 5),
        'Komuna e Parisit': (-10, -10),
        'Blloku': (0, -15),
        'Liqeni': (5, -20)
    }

    # Emergency handling times
    time_on_scene = {
        'Pazari i Ri': 120,  # Fire
        'Sheshi Skënderbej': 45,  # Accident
        'Liqeni': 90,  # Flood
        'Komuna e Parisit':120
    }

    # Event labels for output
    event_labels = {
        'Pazari i Ri': 'Fire at Pazari i Ri',
        'Sheshi Skënderbej': 'Accident at Sheshi Skënderbej',
        'Liqeni': 'Flood near Liqeni',
        'Komuna e Parisit': 'Fire at Komuna e Parisit'
    }

    emergency_sites = {'Pazari i Ri', 'Sheshi Skënderbej', 'Liqeni','Komuna e Parisit'}
    start_station = 'Stacioni Qendror'

    problem = FireResponseSolver(
        road_network_tirana, time_on_scene,
        locations_tirana, emergency_sites,
        start_station, event_labels
    )

    # Run UCS
    print("     UCS      ")
    res_ucs = solve(problem, algorithm='ucs')
    print("cost:", res_ucs['cost'], "nodes:", res_ucs['nodes_visited'], "time_ms:", round(res_ucs['time_ms'], 3))

    # Trying the A* algorithm
    print("\n    A*      ")
    res_astar = solve(problem, algorithm='a_star', heuristic=problem.heuristic_a_star)
    print("cost:", res_astar['cost'], "nodes:", res_astar['nodes_visited'], "time_ms:", round(res_astar['time_ms'], 3))

    # Print A* path
    print("\nPath (A*):")
    if res_astar['path']:
        for s in res_astar['path']:
            loc, handled = s
            labels = [problem.event_labels.get(h, h) for h in handled]
            print(f"Location: {loc}, handled: {labels}")


