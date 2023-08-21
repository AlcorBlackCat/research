#!/usr/bin/env python3 
# coding: utf-8

import xml.etree.ElementTree as ET
import sys
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import math
import copy
import csv
from matplotlib.animation import FuncAnimation

from animation import *
from functions import *
from Car import Car, fake_Car
from Obstacle import Obstacle, fake_Obstacle
from lane import Lane
from grid_road_segment import RoadSegment

root = read_parse_netxml(infilename)
x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list = create_road_network(root)
road_segments_list = create_road_segments(edge_lanes_list)

edges_all_list = DG.edges()
edges_cars_dic = {}
edges_obstacles_dic = {}

for item in edges_all_list:
    edges_obstacles_dic[ item ] = []
    edges_cars_dic[ item ] = []

create_obstacles(number_of_obstacles, number_of_fake_cars, having_fake_obstacle,edge_lanes_list, x_y_dic)

create_cars(number_of_cars, number_of_fake_cars)

plot_car_and_obstacle(cars_list,edges_cars_dic, sensitivity, lane_dic, edge_length_dic,car_forward_pt,diff_dist,obstacle_list,fakeobstacle_list,edge_lanes_list, x_y_dic, obstacle_node_id_list)


