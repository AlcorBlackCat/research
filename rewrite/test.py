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
from Car import *
from Obstacle import *
from lane import *
from grid_road_segment import *

root = read_parse_netxml(infilename)
x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list = create_road_network(root)
road_segments_list = create_road_segments(edge_lanes_list)

draw_road_network(DG)
plt.show()
