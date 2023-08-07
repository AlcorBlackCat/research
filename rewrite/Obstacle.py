import networkx as nx
import numpy as np
import math

class Obstacle:
    def __init__(self, obstacle_node_id, obstacle_lane_id, pass_jdg):  #pass_jdg = Pass judgment(通行判定)
        #self.current_position = []   #できればmainで書くときに省略したい
        self.obstacle_node_id = obstacle_node_id
        self.obstacle_lane_id = obstacle_lane_id
        self.pass_jdg = pass_jdg

class fake_Obstacle(Obstacle):
    pass
