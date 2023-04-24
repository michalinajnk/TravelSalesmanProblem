import random
import time
from TSP import TSP


def generate_random_vals(seedValue, nVal):
    random.seed(seedValue)
    n = nVal
    adj_matrix = [[0] * n for _ in range(n)]  # initialize matrix with zeros

    for i in range(n):
        for j in range(n):
            if i != j:
                weight = random.randint(1, 30)  # generate random weight
                adj_matrix[i][j] = weight  # set weight in matrix
    return n, adj_matrix


if __name__ == '__main__':
    # array which contains number odf cities to traverse
    times = []
    no_cities_range = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    k_value_range = [2, 3, 4, 5]

    print("Solution for B&S: \n")
    for i in no_cities_range:
        no_cities, distances = generate_random_vals(0, i)
        tsp = TSP(no_cities, distances)
        for j in k_value_range:
            start_time = time.time()
            best_path, upperBound = tsp.solve_b_and_s(j)
            # time_measurement
            elapsed_time = time.time() - start_time
            times.append(elapsed_time)
            print("Elapsed time -  " + str(elapsed_time))

    print("Solution for B&B: \n")
    for i in no_cities_range:
        start_time = time.time()
        no_cities, distances = generate_random_vals(0, i)
        tsp = TSP(no_cities, distances)
        best_path, upperBound = tsp.solve_b_and_b()
        # time_measurement
        elapsed_time = time.time() - start_time
        times.append(elapsed_time)
        print("Elapsed time -  " + str(elapsed_time))
