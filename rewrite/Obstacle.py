import networkx as nx
import numpy as np
import math

class Obstacle:
    def __init__(self, obstacle_node_id, obstacle_lane_id): 
        self.current_position = []  
        self.obstacle_node_id = obstacle_node_id
        self.obstacle_lane_id = obstacle_lane_id

    def init(self, DG):
        print(self.obstacle_node_id)
        self.current_position = DG.nodes[ self.obstacle_node_id ]["pos"]

class fake_Obstacle(Obstacle):
    pass
