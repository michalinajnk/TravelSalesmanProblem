import random
import time
from TSP import TSP
def generate_randomVals(seedValue, nVal):
  #  random.seed(seedValue)
    n = nVal
    adj_matrix = [[0] * n for _ in range(n)]  # initialize matrix with zeros

    for i in range(n):
        for j in range(n):
            if i != j:  # avoid self-loops
                weight = random.randint(1, 30)  # generate random weight
                adj_matrix[i][j] = weight  # set weight in matrix
    return n, adj_matrix


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # array which contains number odf cities to traverse
    times = []
    no_cities_range = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    k_value_range = [2, 3, 4, 5]

    print("Solution for B&S: \n")
    for i in no_cities_range:
        no_cities, distances = generate_randomVals(0, i)
        tsp = TSP(no_cities, distances)
        for j in k_value_range:
            start_time = time.time()
            best_path, upperBound = tsp.solve_B_and_S(j)
            # time_measurment
            elapsed_time = time.time() - start_time
            times.append(elapsed_time)
            print("Elapsed time -  " + str(elapsed_time))

    print("Solution for B&B: \n")
    for i in no_cities_range:
        start_time = time.time()
        no_cities, distances = generate_randomVals(0, i)
        tsp = TSP(no_cities, distances)
        best_path, upperBound = tsp.solve_B_and_B()
        #ime_measurment
        elapsed_time = time.time() - start_time
        times.append(elapsed_time)
        print("Elapsed time -  " + str(elapsed_time))






