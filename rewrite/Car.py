#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import networkx as nx
import numpy as np
import math
import copy
import pprint

from functions import *

class Car:
    def __init__(self, orig_node_id, dest_node_id, dest_lane_id, shortest_path, current_lane_id, DG ):
        self.orig_node_id  = orig_node_id #起点
        self.dest_node_id  = dest_node_id #終点
        self.dest_lane_id = dest_lane_id    #道路
        self.shortest_path = shortest_path #最短経路
        self.current_lane_id =  current_lane_id #現在のレーン
        self.current_sp_index = 0 #最短経路のindex
        self.current_speed = 0.0
        self.current_start_node = []   #最新または現在の開始位置
        self.current_position = []   #最新または現在の位置
        self.current_end_node = []    #最新または現在の終了位置
        self.obstacles_info_list = []   #通行不能所
        self.current_distance = 0.0   #最新または現在の道のり、距離
        self.number_of_shortest_path_changes = 0   #最短経路の変更数
        #self.number_of_opportunistic_communication = 0    #すれ違い通信数 未実装
        self.elapsed_time = 0 #経過時間
        self.moving_distance = 0  #移動距離
        self.goal_arrived = False   #ゴール到着はFalse
        self.DG_copied = copy.deepcopy(DG)   #ディープコピーも一方のオブジェクトをもう一方に複製する動作　　オブジェクトがリストなどの属性を持っていたとして、シャシャローコピーはそのリストのコピーには参照を使うが、ディープコピーはリストの要素を再帰的にコピーして複製　DGをコピーする
        self.opportunistic_communication_frag = True  #すれ違い通信をするかどうか
        self.short_path = []  #最短経路?
        self.obstacle_dic = {}   #通行不能個所の空の辞書？
    

    def init(self, DG):  
        current_start_node_id = self.shortest_path[ self.current_sp_index ]  
        self.current_start_node = DG.nodes[ current_start_node_id ]["pos"]  
        self.current_position = DG.nodes[ current_start_node_id ]["pos"]
        current_end_node_id = self.shortest_path[ self.current_sp_index+1]
        self.current_end_node = DG.nodes[ current_end_node_id ]["pos"]
        current_edge_attributes = DG.get_edge_data(current_start_node_id, current_end_node_id)  
        self.current_max_speed = current_edge_attributes["speed"]  
        self.current_distance = current_edge_attributes["weight"]
    
    # Optimal Velocity Function to determine the current speed　　（現在の速度を決定する最適速度機能）
    #最適速度関数
    def V(self, inter_car_distance):
        return  0.5 * self.current_max_speed * (np.tanh(inter_car_distance - 2) + np.tanh(2)) #車間距離から自身の速度を決定する
    
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
                self.goal_arrived = True
                self.current_position = self.current_end_node
                return self.current_position[0], self.current_position[1], self.goal_arrived, None, None
                
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
                
        return self.current_position[0], self.current_position[1], self.goal_arrived, car_forward_pt, diff_dist
    
    def U_turn(self, edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list):  #Uターンのための関数を定義
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
                self.shortest_path = nx.astar_path(self.DG_copied, current_start_node_id, self.dest_node_id)
                break
            except Exception:
                self.dest_lane_id = np.random.randint(len(edge_lanes_list))
                self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]
                while self.dest_node_id in obstacle_node_id_list or self.current_lane_id == self.dest_lane_id:
                    self.dest_lane_id = np.random.randint(len(edge_lanes_list))
                    self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]

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

    def astar_shortest_path(self, x_y_dic):  #経路再計算
        self.shortest_path = nx.astar_path(self.DG_copied, x_y_dic[(self.current_position[0], self.current_position[1])], self.destination_node_id)



       
    

class fake_Car(Car):  #経路選択にfakeobsの影響を受けない、fakeobsを最初から持ってる仕様に
    #TODO  fake_obstacle_dicの追加、経路計算時にfake_obstacle_dicを除外　obstaclelistからランダムで抽選→fakeobsに変換
    #super().__init__()
    def creat_fake_obstacle():
        fake_obstacle_dic = []
        global fake_obstacles_list
        global having_fake_obstacle

        for j in range(having_fake_obstacle):
            a = np.random.choice(obstacles_list)
            fake_obstacle_dic.append(a)
            fake_obstacles_list.appned(a)  #要print
            obstacles_list.remove(a)

