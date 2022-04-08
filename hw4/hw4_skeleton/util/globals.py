import csv
import os
import numpy as np
class globals:
    def __init__(self):
        self.base_folder = os.getcwd()
        self.m = 3
        self.num_x = 5
        self.num_y = 4
        self.N = 20

        self.pickup_distribution = {}
        with open(self.base_folder+'/../data/pickup_distribution.csv', mode='r') as infile:
            reader = csv.reader(infile)
            self.pickup_distribution = dict(((int(rows[0]), int(rows[1])), float(rows[2])) for rows in reader)

        self.dropoff_distribution = {}
        with open(self.base_folder+'/../data/dropoff_distribution.csv', mode='r') as infile:
            reader = csv.reader(infile)
            self.dropoff_distribution = dict(((int(rows[0]), int(rows[1])), float(rows[2])) for rows in reader)

    def manhattan_distance(self, location1, location2):
        return np.abs(location1[0]-location2[0]) + np.abs(location1[1]-location2[1])