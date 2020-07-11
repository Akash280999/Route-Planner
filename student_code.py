import math as m
from queue import PriorityQueue as PQueue


def get_distance(start, goal):
    # Euclidean distance formula used
    x1, x2 = start[0], goal[0]
    y1, y2 = start[1], goal[1]
    return m.sqrt(m.pow((x1 - x2), 2) + m.pow((y1 - y2), 2))


def get_path(prev, start, goal):
    # Method to return a shortest path in list form
    path = [goal]
    while goal != start:
        goal = prev[goal]
        path.append(goal)
    path.reverse()
    return path


def shortest_path(graph, start, goal):
    queue = PQueue()
    queue.put(start, 0)

    # Dictionary to track previous node
    prev = {start: None}
    # Dictionary to track cost
    cost = {start: 0}

    while queue.empty() == False:
        # get lowest priority node
        curr = queue.get()

        # when lowest priority is at goal node
        if curr == goal:
            get_path(prev, start, goal)

        for next in graph.roads[curr]:
            # Compute distance from current intersection to next intersection
            # current cost + straightline distance cost
            new_cost = cost[curr] + get_distance(graph.intersections[curr], graph.intersections[next])

            if next not in cost or new_cost < cost[next]:
                cost[next] = new_cost
                # Priority is new_cost + straightline distance cost
                priority = new_cost + get_distance(graph.intersections[curr], graph.intersections[next])
                # Push the next_node onto the Priority queue
                queue.put(next, priority)
                prev[next] = curr
    # helper function
    return get_path(prev, start, goal)
