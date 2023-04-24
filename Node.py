import math
class Node:
    def __init__(self, id, path, cost=0, level=0):
        self.id = id
        self.path = path
        self.cost = cost
        self.level = level

    """
    The sum of the weights of the edges in the MST is added to the cost of the current path,
    and the resulting value is returned as the lower bound.
    The MST heuristic is a widely used heuristic for the TSP
    and provides an estimate of the optimal solution.
    """
    def get_lower_bound(self, adj_matrix):
        if self.level == len(adj_matrix) - 1:
            return self.cost + adj_matrix[self.path[-1]][self.path[0]]
        else:
            lb = self.cost
            visited = set(self.path)
            while len(visited) < len(adj_matrix):
                min_cost = math.inf
                min_node = None
                for node in visited:
                    for neighbor, cost in enumerate(adj_matrix[node]):
                        if neighbor not in visited and cost < min_cost:
                            min_cost = cost
                            min_node = neighbor
                visited.add(min_node)
                lb += min_cost
            return lb + adj_matrix[self.path[-1]][self.path[0]]

    def get_children(self, adj_matrix):
        children = []
        for i, cost in enumerate(adj_matrix[self.path[-1]]):
            if i not in self.path and cost > 0:
                new_path = self.path + [i]
                if len(new_path) > 1:
                    new_cost = self.cost + adj_matrix[new_path[-2]][i]
                else:
                    new_cost = adj_matrix[self.id][i]
                children.append(Node(i, new_path, new_cost, self.level + 1))
                print("Children :" + str(children))
        return children

