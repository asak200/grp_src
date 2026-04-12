import numpy as np
import math

class BidirectionalRRT:
    def __init__(self, occupancy_grid, start, goal, step_size=15, max_iter=3000):
        self.grid = occupancy_grid
        self.rows, self.cols = occupancy_grid.shape
        self.start = Node(start[1], start[0]) # x, y
        self.goal = Node(goal[1], goal[0])
        self.step_size = step_size
        self.max_iter = max_iter
        
        # Two trees: One growing from Start, one from Goal
        self.start_tree = [self.start]
        self.goal_tree = [self.goal]

    def plan(self):
        for i in range(self.max_iter):
            # 1. Expand Start Tree (Tree A)
            rnd_node = self.get_random_point()
            nearest_A = self.get_nearest_node(self.start_tree, rnd_node)
            new_node_A = self.steer(nearest_A, rnd_node)

            if self.is_safe(new_node_A) and self.check_line_of_sight(nearest_A, new_node_A):
                self.start_tree.append(new_node_A)
                
                # 2. Try to connect Goal Tree (Tree B) directly to this new node
                nearest_B = self.get_nearest_node(self.goal_tree, new_node_A)
                new_node_B = self.steer(nearest_B, new_node_A)

                if self.is_safe(new_node_B) and self.check_line_of_sight(nearest_B, new_node_B):
                    self.goal_tree.append(new_node_B)
                    
                    # 3. CHECK CONNECTIVITY
                    # If the two trees are close enough to touch
                    dist = math.hypot(new_node_A.x - new_node_B.x, new_node_A.y - new_node_B.y)
                    if dist <= self.step_size:
                        print(f"✅ Bi-RRT Connected in {i} iterations!")
                        return self.generate_final_path(new_node_A, new_node_B)

            # 4. Swap Trees (Grow from the other side next time to balance it out)
            if len(self.start_tree) > len(self.goal_tree):
                self.start_tree, self.goal_tree = self.goal_tree, self.start_tree

        return None # Failed

    def steer(self, from_node, to_node):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = Node(from_node.x + self.step_size * math.cos(theta),
                        from_node.y + self.step_size * math.sin(theta))
        new_node.parent = from_node
        return new_node

    def get_random_point(self):
        return Node(np.random.randint(0, self.cols), np.random.randint(0, self.rows))

    def get_nearest_node(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        min_index = dlist.index(min(dlist))
        return node_list[min_index]

    def is_safe(self, node):
        if node.x < 0 or node.x >= self.cols or node.y < 0 or node.y >= self.rows:
            return False
        try:
            # Check grid (y, x)
            if self.grid[int(node.y)][int(node.x)] == 1:
                return False
        except IndexError:
            return False
        return True

    def check_line_of_sight(self, node1, node2):
        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = int(dist / 2) # Check every 2 pixels
        if steps == 0: return True

        for i in range(steps):
            t = i / steps
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            if self.grid[int(y)][int(x)] == 1:
                return False
        return True

    def generate_final_path(self, meet_node_A, meet_node_B):
        # 1. Path from Start -> Meeting Point
        path_start = []
        node = meet_node_A
        while node is not None:
            path_start.append([node.x, node.y])
            node = node.parent
        path_start = path_start[::-1] # Reverse to get Start -> Meet

        # 2. Path from Goal -> Meeting Point
        path_goal = []
        node = meet_node_B
        while node is not None:
            path_goal.append([node.x, node.y])
            node = node.parent
        
        # 3. Stitch them together
        # Check which tree was which (since we swapped them randomly)
        # We check the first node of path_start. If it's near the original start, good.
        # Otherwise, flip the logic.
        
        full_path = path_start + path_goal
        return full_path

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None