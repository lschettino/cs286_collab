import pandas as pd
from numpy import genfromtxt
import sys, os
sys.path.append('../util/')
from graphics import graphics, plot_distribution
import csv


def find_pickup_probability():
    time_pickup_dropoff_data = pd.read_csv(os.getcwd()+'/../data/requests_details_day1.csv', names=['pickup_minute', 'pickup_x', 'pickup_y', 'dropoff_x', 'dropoff_y'])
    # print('show file content')
    # for index, row in time_pickup_dropoff_data.iterrows():
    #     min = row['pickup_minute']
    #     pu_x = row['pickup_x']
    #     pu_y = row['pickup_y']
    #     do_x = row['dropoff_x']
    #     do_y = row['dropoff_y']
    #     print(min, pu_x, pu_y, do_x, do_y)
    pickup_distribution = {}
    # The key (x,y) : (int, int). Pickup location's x, and y coordinates
    # the value p: float. p is the pickup probability of location (x,y)
    # sum(pickup_distribution.values()) should be 1
    ################################# Begin your code ###############################

    ################################# End your code #################################
    plot_distribution(pickup_distribution, "Pickup distribution")
    #print(pickup_distribution)
    f = open(os.getcwd() + '/../data/pickup_distribution.csv', 'w')
    for (x, y) in pickup_distribution.keys():
        f.write(str(x) + ',' + str(y) + ',' + str(pickup_distribution[(x, y)]) + '\n')
    f.close()
    #return pickup_distribution

def find_dropoff_probability():
    time_pickup_dropoff_data = pd.read_csv(os.getcwd()+'/../data/requests_details_day1.csv', names=['pickup_minute', 'pickup_x', 'pickup_y', 'dropoff_x', 'dropoff_y'])
    dropoff_distribution = {} #The key should be a tuple of (x,y) and the value would be the dropoff probability of that location
    # The key (x,y) : (int, int). Dropoff location's x, and y coordinates
    # the value p: float. p is the dropoff probability of location (x,y)
    # sum(dropoff_distribution.values()) should be 1
    ################################# Begin your code ###############################

    ################################# End your code #################################
    #return dropoff_distribution
    plot_distribution(dropoff_distribution, "Dropoff distribution")
    #print(dropoff_distribution)
    f = open(os.getcwd() + '/../data/dropoff_distribution.csv', 'w')
    for (x, y) in dropoff_distribution.keys():
        f.write(str(x) + ',' + str(y) + ',' + str(dropoff_distribution[(x, y)]) + '\n')
    f.close()







