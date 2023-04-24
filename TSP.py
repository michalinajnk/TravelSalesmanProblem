import math
from queue import PriorityQueue
from Node import Node


class TSP:
    def __init__(self, noCities, adjacency_matrix):
        self.adjacencyMatrix = adjacency_matrix
        self.noCities = noCities
        self.upperBound = self.calculate_upper_bound()
        self.best_path = []

    """
    DFS in order to find a first cycle and set its cost to a initial value for an upper bound
    """
    def traverse_tree(self, node):
        if not node.get_children(self.adjacencyMatrix):
            return node.cost
        for child in node.get_children(self.adjacencyMatrix):
            result = self.traverse_tree(child)
            if result is not None and result < self.upperBound:
                self.upperBound = result
        return None

    def calculate_upper_bound(self):
        start_node = 0
        path = [start_node]
        cost = 0
        while len(path) < self.noCities:
            min_cost = math.inf
            min_node = None
            for i, weight in enumerate(self.adjacencyMatrix[path[-1]]):
                if i not in path and weight < min_cost:
                    min_cost = weight
                    min_node = i
            path.append(min_node)
            cost += min_cost
        cost += self.traverse_tree(Node(0, path, cost))
        print(str(path))
        print("Upper bound :" + str(cost))
        return cost

    def get_neighbors(self, node_index, k=-1):
        neighbors = []
        for i, weight in enumerate(self.adjacencyMatrix[node_index]):
            if weight != 0:
                neighbors.append(i)
        return neighbors

    def get_k_neighbors(self, node_index, k=3):
        neighbors = []
        for i, weight in enumerate(self.adjacencyMatrix[node_index]):
            if weight != 0:
                neighbors.append((i, weight))  # Add neighbor and its weight as a tuple
        neighbors.sort(key=lambda x: x[1])  # Sort neighbors by weight
        k = min(k, len(neighbors))
        k_neighbors = [neighbor[0] for neighbor in neighbors[:k]]  # Extract the indices of the first k neighbors
        return k_neighbors

    def solve_b_and_b(self):
        return self.solve_without_comments(self.get_neighbors)
        # return self.solve(self.get_neighbors)

    def solve_b_and_s(self, k=3):
        return self.solve_without_comments(self.get_k_neighbors, k)
        # return self.solve(self.get_K_neighbors, k)

    def solve(self, get_neighbors_func, k=-1):
        # Number of nodes in the graph
        num_nodes = self.noCities
        print("Number of nodes in the tree: " + str(num_nodes))
        pq = PriorityQueue()

        # Initialize the priority queue with children of the root node
        for i in range(1, self.noCities - 1):
            startList = [0, i]
            node = Node(i, startList)
            # we negate the priority value in order to prioritize nodes with smaller lower bound
            pq.put((-1 * node.get_lower_bound(self.adjacencyMatrix), startList))

        print("Priority queue: " + str(pq))

        # Loop until the priority queue is empty
        while not pq.empty():

            # Get the node with the lowest lower bound from the priority queue and its current path
            lowerBound, path = pq.get()
            # we negate the lowerBound value to get a real value of lower bound for a node
            # reversing the process of setting a priority to a node)
            lowerBound = lowerBound * (-1)

            # If the lower bound is greater than or equal to the upper bound, we can prune this node
            if lowerBound >= self.upperBound:
                print("Pruning of the branch, not worth to go further, Cost: " + str(
                    lowerBound) + ", Upper bound: " + str(self.upperBound))
                continue

            # If the path of the node includes all nodes, we have found a complete tour
            if len(path) == num_nodes:
                path.append(0)
                print("We have already traversed all nodes Num of all Nodes: " + str(num_nodes))
                # Calculate the cost of the complete tour
                lowerBound += self.adjacencyMatrix[path[-1]][path[0]]

                # If the lowerBound is smaller than the current upper bound, update the best path and upper bound
                if lowerBound < self.upperBound:
                    print("Updating the best current path")
                    self.best_path = path
                    self.upperBound = lowerBound

            # If the path of the node does not include all nodes, expand the node
            else:
                # Get the neighbors of the last node in the path using the passed get_neighbors_func
                neighbors = get_neighbors_func(path[-1], k)
                print("Neighbors of the last visited: " + str(neighbors))

                # Loop over the neighbors and create child nodes
                for neighbor in neighbors:
                    # If the neighbor is already in the path, skip it
                    print(
                        "Checking if neighbor in the path already -  " + str(neighbor in path) + ", Neighbor - " + str(
                            neighbor) + ", Path " + str(path))
                    if neighbor in path:
                        continue

                    # Calculate the new lower bound for the child, by adding the neighbor to the path
                    new_lowerBound = lowerBound + self.adjacencyMatrix[path[-1]][neighbor]

                    # If the lower bound of the child is lower than the upper bound, add it to the priority queue
                    if new_lowerBound < self.upperBound:
                        print("Updating queue: new Lower Bound - " + str(new_lowerBound) + ", Upper bound - " + str(
                            self.upperBound))
                        pq.put((-1 * new_lowerBound, path + [neighbor]))

        # Return the best path and the cost of that path
        print("Adjacency Matrix:")
        for i in range(self.noCities):
            for j in range(self.noCities):
                print(self.adjacencyMatrix[i][j], end="\t")  # use '\t' to separate elements
            print()  # print a new line after each row
        print("Best Path - " + str(self.best_path) + ", Cost - " + str(self.upperBound))
        return self.best_path, self.upperBound

    def solve_without_comments(self, get_neighbors_func, k=-1):
        # Number of nodes in the graph
        num_nodes = self.noCities

        pq = PriorityQueue()

        # Initialize the priority queue with children of the root node
        for i in range(1, self.noCities - 1):
            startList = [0, i]
            node = Node(i, startList)
            pq.put((node.get_lower_bound(self.adjacencyMatrix), startList))

        # Loop until the priority queue is empty
        while not pq.empty():

            # Get the node with the lowest lower bound from the priority queue and its current path
            lowerBound, path = pq.get()

            # If the lower bound is greater than or equal to the upper bound, we can prune this node
            if lowerBound >= self.upperBound:
                continue

            # If the path of the node includes all nodes, we have found a complete tour
            if len(path) == num_nodes:
                path.append(0)

                # Calculate the cost of the complete tour
                lowerBound += self.adjacencyMatrix[path[-1]][path[0]]

                # If the lowerBound is smaller than the current upper bound, update the best path and upper bound
                if lowerBound < self.upperBound:
                    self.best_path = path
                    self.upperBound = lowerBound

            # If the path of the node does not include all nodes, expand the node
            else:
                # Get the neighbors of the last node in the path using the passed get_neighbors_func
                neighbors = get_neighbors_func(path[-1], k)

                # Loop over the neighbors and create child nodes
                for neighbor in neighbors:

                    # If the neighbor is already in the path, skip it
                    if neighbor in path:
                        continue

                    # Calculate the new lower bound for the child, by adding the neighbor to the path
                    new_lowerBound = lowerBound + self.adjacencyMatrix[path[-1]][neighbor]

                    # If the lower bound of the child is lower than the upper bound, add it to the priority queue
                    if new_lowerBound < self.upperBound:
                        pq.put((new_lowerBound, path + [neighbor]))

        # Return the best path and the cost of that path
        print("Adjacency Matrix:")
        for i in range(self.noCities):
            for j in range(self.noCities):
                print(self.adjacencyMatrix[i][j], end="\t")  # use '\t' to separate elements
            print()  # print a new line after each row
        print("Best Path - " + str(self.best_path) + ", Cost - " + str(self.upperBound))
        return self.best_path, self.upperBound
