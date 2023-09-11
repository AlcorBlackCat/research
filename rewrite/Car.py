#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import networkx as nx
import numpy as np
import math
import copy
import pprint

class Car:
    def __init__(self, origin_node_id, destination_node_id, destination_lane_id, shortest_path, current_lane_id, edges_cars_dic, DG ):
        self.origin_node_id  = origin_node_id 
        self.destination_node_id  = destination_node_id 
        self.destination_lane_id = destination_lane_id    
        self.shortest_path = shortest_path 
        self.current_lane_id =  current_lane_id 
        #self.edges_cars_dic = edges_cars_dic
        self.current_sp_index = 0 
        self.current_speed = 0.0
        self.current_start_node = []   
        self.current_position = []   
        self.current_end_node = []   
        self.obstacles_info_list = []  
        self.current_distance = 0.0  
        self.number_of_shortest_path_changes = 0  
        #self.number_of_opportunistic_communication = 0    #すれ違い通信数 未実装
        self.elapsed_time = 0
        self.moving_distance = 0 
        self.goal_arrived_flag = False  
        self.DG_copied = copy.deepcopy(DG)  
        self.opportunistic_communication_frag = True 
        self.short_path = [] 
        self.obstacle_dic = {}   
    

    def init(self, DG):  
        current_start_node_id = self.shortest_path[ self.current_sp_index ]  
        self.current_start_node = DG.nodes[ current_start_node_id ]["pos"]  
        self.current_position = DG.nodes[ current_start_node_id ]["pos"]
        current_end_node_id = self.shortest_path[ self.current_sp_index+1]
        self.current_end_node = DG.nodes[ current_end_node_id ]["pos"]
        current_edge_attributes = DG.get_edge_data(current_start_node_id, current_end_node_id)  
        self.current_max_speed = current_edge_attributes["speed"]  
        self.current_distance = current_edge_attributes["weight"]
    

    #最適速度関数
    def V(self, inter_car_distance):
        return  0.5 * self.current_max_speed * (np.tanh(inter_car_distance - 2) + np.tanh(2)) 
    
    def update_current_speed(self, sensitivity, inter_car_distance):
        self.current_speed += sensitivity * (self.V(inter_car_distance) - self.current_speed)

    def move(self, edges_cars_dic, sensitivity, lane_dic, edge_length_dic):
        self.elapsed_time += 1
        
        direction_x = self.current_end_node[0] - self.current_position[0]
        direction_y = self.current_end_node[1] - self.current_position[1]
        arg = math.atan2(direction_y, direction_x)
        
        if np.sqrt((self.current_position[0] - self.current_end_node[0])**2 + (self.current_position[1] - self.current_end_node[1])**2) < self.current_speed:
            # Reached the end of the lane, move to the next lane or destination
            if self.current_sp_index + 1 >= len(self.shortest_path):
                self.goal_arrived_flag = True
                self.current_position = self.current_end_node
                return self.current_position[0], self.current_position[1], self.goal_arrived_flag, None, None
                
            self.current_sp_index += 1
            current_start_node_id = self.shortest_path[self.current_sp_index - 1]
            current_end_node_id = self.shortest_path[self.current_sp_index]
            edges_cars_dic[(current_start_node_id, current_end_node_id)].remove(self)
            
            if self.shortest_path[self.current_sp_index] in lane_dic:
                self.moving_distance += edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]]
                
            current_start_node_id = self.shortest_path[self.current_sp_index]
            current_end_node_id = self.shortest_path[self.current_sp_index + 1]
            self.current_start_node = self.DG_copied.nodes[current_start_node_id]["pos"]
            self.current_position = self.DG_copied.nodes[current_start_node_id]["pos"]
            self.current_end_node = self.DG_copied.nodes[current_end_node_id]["pos"]
            edges_cars_dic[(current_start_node_id, current_end_node_id)].append(self)
            
            if edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) > 0:
                car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) - 1
                car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
                diff_dist = 50
            else:
                car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
                car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
                diff_dist = 50.0
        else:
            x_new = self.current_position[0] + self.current_speed * np.cos(arg)
            y_new = self.current_position[1] + self.current_speed * np.sin(arg)
            self.current_position = [x_new, y_new]
            
            current_start_node_id = self.shortest_path[self.current_sp_index]
            current_end_node_id = self.shortest_path[self.current_sp_index + 1]
            
            if edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) > 0:
                car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) - 1
                car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
                diff_dist = np.sqrt((car_forward_pt.current_position[0] - self.current_position[0])**2 + (car_forward_pt.current_position[1] - self.current_position[1])**2)
            else:
                car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
                car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
                diff_dist = 50.0
                
            self.update_current_speed(sensitivity, diff_dist)
            
            if self.shortest_path[self.current_sp_index] not in lane_dic:
                None
            else:
                self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]
                
        return self.current_position[0], self.current_position[1], self.goal_arrived_flag, car_forward_pt, diff_dist
    
    def U_turn(self, edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list): 
        self.current_sp_index += 1
        x_new = self.current_end_node[0]
        y_new = self.current_end_node[1]

        current_start_node_id = self.shortest_path[self.current_sp_index - 1]
        current_end_node_id = self.shortest_path[self.current_sp_index]
        pre_start_node_id = current_start_node_id
        pre_end_node_id = current_end_node_id

        #障害物情報の追加
        if current_end_node_id not in self.obstacles_info_list:
            self.obstacles_info_list.append(current_end_node_id)
        
        #現在の車線id
        self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]

        #対向車線を求め, 始点,終点,現在地を求める
        for i in range(len(edge_lanes_list) - 1):
            for j in range(i + 1, len(edge_lanes_list)):
                if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
                    if edge_lanes_list[self.current_lane_id] == edge_lanes_list[i]:
                        current_start_node_id = x_y_dic[(edge_lanes_list[j].node_x_list[0], edge_lanes_list[j].node_y_list[0])]
                        current_end_node_id = x_y_dic[(edge_lanes_list[j].node_x_list[-1], edge_lanes_list[j].node_y_list[-1])]
                    elif edge_lanes_list[self.current_lane_id] == edge_lanes_list[j]:
                        current_start_node_id = x_y_dic[(edge_lanes_list[i].node_x_list[0], edge_lanes_list[i].node_y_list[0])]
                        current_end_node_id = x_y_dic[(edge_lanes_list[i].node_x_list[-1], edge_lanes_list[i].node_y_list[-1])]

        self.current_start_node = self.DG_copied.nodes[current_start_node_id]["pos"]
        self.current_position = self.DG_copied.nodes[current_start_node_id]["pos"]
        self.current_end_node = self.DG_copied.nodes[current_end_node_id]["pos"]

        #障害物を含む辺の削除？
        for i in obstacles_list:
            if self.DG_copied.has_edge(*i):
                self.DG_copied.remove_edge(*i)

        while True:
            try:
                self.shortest_path = nx.astar_path(self.DG_copied, current_start_node_id, self.destination_node_id)
                break
            except Exception:
                self.destination_lane_id = np.random.randint(len(edge_lanes_list))
                self.destination_node_id = x_y_dic[(edge_lanes_list[self.destination_lane_id].node_x_list[-1], edge_lanes_list[self.destination_lane_id].node_y_list[-1])]
                while self.destination_node_id in obstacle_node_id_list or self.current_lane_id == self.destinaton_lane_id:
                    self.destination_lane_id = np.random.randint(len(edge_lanes_list))
                    self.destination_node_id = x_y_dic[(edge_lanes_list[self.destination_lane_id].node_x_list[-1], edge_lanes_list[self.destination_lane_id].node_y_list[-1])]

        self.number_of_shortest_path_changes += 1
        self.current_sp_index = 0

        current_start_node_id = self.shortest_path[self.current_sp_index]
        self.current_start_node = self.DG_copied.nodes[current_start_node_id]["pos"]
        self.current_position = self.DG_copied.nodes[current_start_node_id]["pos"]
        current_end_node_id = self.shortest_path[self.current_sp_index + 1]
        self.current_end_node = self.DG_copied.nodes[current_end_node_id]["pos"]
        current_edge_attributes = self.DG_copied.get_edge_data(current_start_node_id, current_end_node_id)
        self.current_max_speed = current_edge_attributes["speed"]
        self.current_distance = current_edge_attributes["weight"]

        return x_new, y_new
       
    def trade_obstacle_info(self, obstacle_dic,car_info):
        self.obstacle_dic = obstacle_dic
        comm_target = car_info
        if type(cars_list[comm_target]) == Car:
            for i in self.obstacle_dic.key():
                if i not in comm_target.obstacle_dic:
                    comm_target.obstacle_dic.append(i) #相手が持ってる障害物情報に追加
                else:
                    pass

            for j in comm_target.obstacle_dic.key():
                if j not in self.obstacle_dic:
                    self.obstacle_dic.append(j)  #自分が持ってる障害物情報に追加
                else:
                    pass
        
        if type(cars_list[comm_target]) == fake_Car:
            for i in self.obstacle_dic.key():  #相手の辞書に追加
                if i not in comm_target.obstacle_dic and i not in comm_target.fake_obstacle_dic:
                    comm_target.obstacle_dic.append(i)
                else:
                    pass
            
            for j in comm_target.obstacle_dic.key() :  #自分の方に追加
                if j not in self.obstacle_dic:
                    self.obstacle_dic.append(j)
                else:
                    pass
            for k in comm_target.fake_obstacle_dic.key():  #通信する車両がもっているニセの通行不能箇所情報を自分が持ってる辞書を比較し、自車が持っていないなら追加
                if k not in self.obstacle_dic:
                    self.obstacle_dic.append(k)
                else:
                    pass

    def astar_shortest_path(self):  #経路再計算
        self.shortest_path = nx.astar_path(self.DG_copied, x_y_dic[(self.current_position[0], self.current_position[1])], self.destination_node_id)



       
    

class fake_Car(Car):  #経路選択にfakeobsの影響を受けない、fakeobsを最初から持ってる仕様に
    #TODO  fake_obstacle_dicの追加、経路計算時にfake_obstacle_dicを除外　obstaclelistからランダムで抽選→fakeobsに変換
    def create_fake_obstacle(self, obstacles_list, fake_obstacles_list, having_fake_obstacle):
        self.fake_obstacles_dic = []

        for j in range(having_fake_obstacle):
            a = np.random.choice(obstacles_list)
            self.fake_obstacles_dic.append(a)
            fake_obstacles_list.append(a) 
            obstacles_list.remove(a)

