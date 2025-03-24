import heapq

class DijkstraGlobalPlanner:
    def __init__(self, points, num_intermediate=5):
        self.obstacles = points
        self.num_intermediate = num_intermediate
    
    def _neighbors(self, node):
        x, y = node
        step_size = 0.5  # Adjustable step size
        neighbors = [(x + step_size, y), (x - step_size, y), 
                     (x, y + step_size), (x, y - step_size)]
        return [n for n in neighbors if n not in self.obstacles]
    
    def find_path(self, start, goal):
        pq = []  # Priority queue for Dijkstra
        heapq.heappush(pq, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        print(4)
        
        while pq:
            print(5)
            current_cost, current_node = heapq.heappop(pq)
            if current_node == goal:
                print(6)
                break  # Goal reached
            
            for neighbor in self._neighbors(current_node):

                print(7)
                new_cost = cost_so_far[current_node] + ((neighbor[0] - current_node[0])**2 + (neighbor[1] - current_node[1])**2) ** 0.5
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    print(8)
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost
                    heapq.heappush(pq, (priority, neighbor))
                    came_from[neighbor] = current_node
        
        return self._sample_intermediate_points(self._reconstruct_path(came_from, start, goal))
    
    def _reconstruct_path(self, came_from, start, goal):
        path = []
        current = goal
        while current != start:
            if current not in came_from:
                return []  # No valid path
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    
    def _sample_intermediate_points(self, path):
        if len(path) <= self.num_intermediate + 2:
            return path  # Return full path if it's short
        sampled_path = [path[0]]
        step = len(path) // (self.num_intermediate + 1)
        for i in range(1, self.num_intermediate + 1):
            sampled_path.append(path[i * step])
        sampled_path.append(path[-1])
        return sampled_path

import numpy as np

class SimplePathPlanner:
    def __init__(self, obstacles, num_intermediate=5, safe_distance=0.3):
        # Convert obstacles to a set of tuples
        self.obstacles = set(tuple(obst) for obst in obstacles)  
        self.num_intermediate = num_intermediate
        self.safe_distance = safe_distance

    def is_collision(self, point):
        return point in self.obstacles

    def adjust_point(self, point):
        """Move the point slightly if it collides with an obstacle."""
        x, y = point
        step_size = self.safe_distance  # Small step to move away

        # Try small displacements to find a collision-free spot
        for dx, dy in [(step_size, 0), (-step_size, 0), (0, step_size), (0, -step_size)]:
            new_point = (x + dx, y + dy)
            if new_point not in self.obstacles:
                return new_point  # Found a valid nearby point
        return point  # If no better option, return the same point

    def find_intermediate_goals(self, start, goal):
        waypoints = [start]
        for i in range(1, self.num_intermediate + 1):
            ratio = i / (self.num_intermediate + 1)
            new_x = start[0] + ratio * (goal[0] - start[0])
            new_y = start[1] + ratio * (goal[1] - start[1])

            # If the point is in an obstacle, adjust it
            new_point = (new_x, new_y)
            if self.is_collision(new_point):
                new_point = self.adjust_point(new_point)

            waypoints.append(new_point)

        waypoints.append(goal)
        return waypoints



    
    
