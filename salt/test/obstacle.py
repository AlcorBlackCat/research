import networkx as nx
import numpy as np
import math

class Obstacle:
    def __init__(self, obstacle_node_id, obstacle_lane_id, DG):
        self.current_position = DG.nodes[obstacle_node_id]["pos"]
        self.orig_node_id = obstacle_node_id
        self.current_lane_id = obstacle_lane_id
        DG.nodes[obstacle_node_id]["obj"] = self
        self.DG = DG
        self.current_start_node = []
        self.current_end_node = []
        self.init(DG)

    def init(self, DG):
        current_node_id = self.orig_node_id
        self.current_start_node = DG.nodes[current_node_id]["pos"]
        self.current_end_node = DG.nodes[current_node_id]["pos"]
        for neighbor in DG.neighbors(current_node_id):
            if DG.get_edge_data(current_node_id, neighbor):
                current_edge_attributes = DG.get_edge_data(current_node_id, neighbor)
                break
        else:
            raise ValueError(f"No edges found for node {current_node_id}")
        self.current_end_node = DG.nodes[neighbor]["pos"]

    def move(self):
       x_new = self.current_position[0]
       y_new = self.current_position[1]
       return x_new, y_new

class FakeObstacle(Obstacle):
    def __init__(self, obstacle_node_id, obstacle_lane_id, DG):
        super().__init__(obstacle_node_id, obstacle_lane_id, DG)

    def does_affect_passing(self):
        return False
