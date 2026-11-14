
from collections import deque
import heapq
import itertools
from math import radians, sin, cos, sqrt, atan2, inf


# 1. Define 10 graph nodes with sample coordinates
NODES = {
    "A": (23.7380, 90.3940),
    "B": (23.7460, 90.3970),
    "C": (23.7330, 90.4050),
    "D": (23.7500, 90.4100),
    "E": (23.7560, 90.4000),
    "F": (23.7280, 90.4150),
    "G": (23.7440, 90.4200),
    "H": (23.7600, 90.4180),
    "I": (23.7350, 90.4300),
    "J": (23.7540, 90.4320),
}


# 2. Define graph connections manually
EDGES = [
    ("A", "B"),
    ("A", "C"),
    ("B", "D"),
    ("B", "E"),
    ("C", "D"),
    ("C", "F"),
    ("D", "E"),
    ("D", "G"),
    ("E", "H"),
    ("F", "G"),
    ("F", "I"),
    ("G", "H"),
    ("G", "J"),
    ("H", "J"),
    ("I", "J"),
]


# 3. Calculate distance using Haversine formula
def distance_km(node1, node2):

    lat1, lon1 = NODES[node1]
    lat2, lon2 = NODES[node2]

    lat1, lon1 = radians(lat1), radians(lon1)
    lat2, lon2 = radians(lat2), radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        sin(dlat / 2) ** 2
        + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    )

    return 6371 * 2 * atan2(sqrt(a), sqrt(1 - a))


# 4. Create weighted adjacency list
GRAPH = {node: {} for node in NODES}

for u, v in EDGES:

    weight = distance_km(u, v)

    GRAPH[u][v] = weight
    GRAPH[v][u] = weight


def calculate_path_distance(path):

    total = 0

    for i in range(len(path) - 1):
        total += GRAPH[path[i]][path[i + 1]]

    return round(total, 2)


def make_result(path, visited):

    return {
        "path": path,
        "distance_km": (
            calculate_path_distance(path) if path else None
        ),
        "visited": visited,
    }


def reconstruct_path(previous, start, goal):

    path = [goal]

    while path[-1] != start:

        parent = previous.get(path[-1])

        if parent is None:
            return []

        path.append(parent)

    return list(reversed(path))


# 5. Breadth First Search (BFS)
def bfs(start, goal):

    queue = deque([[start]])
    visited = {start}
    visited_order = []

    while queue:

        path = queue.popleft()
        current = path[-1]

        visited_order.append(current)

        if current == goal:
            return make_result(path, visited_order)

        for neighbor in GRAPH[current]:

            if neighbor not in visited:

                visited.add(neighbor)
                queue.append(path + [neighbor])

    return make_result([], visited_order)


# 6. Depth First Search (DFS)
def dfs(start, goal):

    stack = [[start]]
    visited = set()
    visited_order = []

    while stack:

        path = stack.pop()
        current = path[-1]

        if current in visited:
            continue

        visited.add(current)
        visited_order.append(current)

        if current == goal:
            return make_result(path, visited_order)

        for neighbor in reversed(list(GRAPH[current])):

            if neighbor not in visited:
                stack.append(path + [neighbor])

    return make_result([], visited_order)


# 7. Dijkstra's Algorithm
def dijkstra(start, goal):

    distances = {node: inf for node in GRAPH}
    distances[start] = 0

    previous = {}

    priority_queue = [(0, start)]
    visited_order = []

    while priority_queue:

        current_distance, current = heapq.heappop(
            priority_queue
        )

        if current_distance != distances[current]:
            continue

        visited_order.append(current)

        if current == goal:
            break

        for neighbor, weight in GRAPH[current].items():

            new_distance = current_distance + weight

            if new_distance < distances[neighbor]:

                distances[neighbor] = new_distance
                previous[neighbor] = current

                heapq.heappush(
                    priority_queue,
                    (new_distance, neighbor)
                )

    if distances[goal] == inf:
        return make_result([], visited_order)

    path = reconstruct_path(previous, start, goal)

    return make_result(path, visited_order)


# 8. A* Search Algorithm
def heuristic(node, goal):

    return distance_km(node, goal)


def astar(start, goal):

    counter = itertools.count()

    priority_queue = [
        (heuristic(start, goal), next(counter), 0, start)
    ]

    g_score = {start: 0}
    previous = {}

    visited = set()
    visited_order = []

    while priority_queue:

        f_score, _, current_g, current = heapq.heappop(
            priority_queue
        )

        if current in visited:
            continue

        if current_g != g_score.get(current):
            continue

        visited.add(current)
        visited_order.append(current)

        if current == goal:
            path = reconstruct_path(previous, start, goal)
            return make_result(path, visited_order)

        for neighbor, weight in GRAPH[current].items():

            tentative_g = current_g + weight

            if tentative_g < g_score.get(neighbor, inf):

                previous[neighbor] = current
                g_score[neighbor] = tentative_g

                f = tentative_g + heuristic(neighbor, goal)

                heapq.heappush(
                    priority_queue,
                    (f, next(counter), tentative_g, neighbor)
                )

    return make_result([], visited_order)


# 9. Select algorithm
def find_path(start, goal, algorithm):

    if start not in GRAPH or goal not in GRAPH:
        raise ValueError("Invalid source or destination")

    algorithms = {
        "bfs": bfs,
        "dfs": dfs,
        "dijkstra": dijkstra,
        "astar": astar,
    }

    if algorithm not in algorithms:
        raise ValueError("Invalid algorithm selected")

    return algorithms[algorithm](start, goal)
