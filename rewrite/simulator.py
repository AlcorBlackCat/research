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

import functions
from grid_car import Car, fakeCar
from obstacle import Obstacle, fakeObstacle
from lane import Lane
from grid_road_segment import RoadSegment

root = read_parse_netxml(infilename)
