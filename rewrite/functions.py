#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#関数を以下に集約
#目次
#8 グローバル変数

import xml.etree.ElementTree as ET
import sys
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import math
import copy
import csv
from matplotlib.animation import FuncAnimation

from Car import Car, fake_Car
from Obstacle import Obstacle, fake_Obstacle
from lane import Lane
from grid_road_segment import RoadSegment

infilename = "grid5x5.net.xml"

number_of_cars = 300 
number_of_obstacles = 10 
number_of_fake_cars = 1 
having_fake_obstacle = 1   #車が持つ偽の通行不能箇所数
opportunistic_communication_rate = 1.0  
sensitivity = 1.0

np.random.seed(123456)

obstacles_list = []
fake_obstacles_list = []
obstacle_node_id_list = []
fakeobs_node_id_list = []
pair_node_id_list = []
fakepair_node_id_list = []
cars_list = []
fakecars_list = []
obstacle_dic = {}

goal_time_list = []
number_of_shortest_path_changes_list = []
moving_distance_list = []
time_list = []

x_y_dic = {}
edge_lanes_list = []
edges_obstacles_dic = {}
edges_cars_dic = {}

DG = nx.DiGraph()

def read_parse_netxml(infilename):
  # open file
  infile = open(infilename, "r")

  # parsing xml
  root = ET.fromstring(infile.read())
  #print(root.tag, root.attrib)
  return root

def create_road_network(root):
  # read edge tagged data for reading the road network
  # create data structure of road network using NetworkX
  global x_y_dic # input: node's x,y pos, output: node id
  lane_dic = {}
  edge_length_dic = {}
  node_id = 0
  lane_id = 0

  DG = nx.DiGraph() # Directed graph of road network
  global edge_lanes_list # list of lane instances
  for child in root:
    if child.tag == "edge":
      lane = Lane()
      if "from" in child.attrib and "to" in child.attrib:
        lane.add_from_to(child.attrib["from"], child.attrib["to"])

      for child2 in child:
        data_list  = child2.attrib["shape"].split(" ")
        node_id_list = []
        node_x_list = []; node_y_list = []
        distance_list = []
        data_counter = 0

        for data in data_list:
          node_x_list.append( float(data.split(",")[0]) )
          node_y_list.append( float(data.split(",")[1]) )
          if (float(data.split(",")[0]), float(data.split(",")[1])) not in x_y_dic.keys():
            node_id_list.append(node_id)
            DG.add_node(node_id, pos=(float(data.split(",")[0]), float(data.split(",")[1])))
            x_y_dic[ (float(data.split(",")[0]), float(data.split(",")[1])) ] = node_id
            node_id += 1

          else:
            node_id_list.append( x_y_dic[ (float(data.split(",")[0]), float(data.split(",")[1])) ] )

          if data_counter >= 1:
            distance_list.append( np.sqrt( (float(data.split(",")[0]) - old_node_x)**2 + (float(data.split(",")[1]) - old_node_y)**2) )
          old_node_x = float(data.split(",")[0])
          old_node_y = float(data.split(",")[1])
          data_counter += 1
        for i in range(len(node_id_list)-1):
          DG.add_edge(node_id_list[i], node_id_list[i+1], weight=distance_list[i], color="black", speed=float(child2.attrib["speed"])) # calculate weight here
        if "from" in child.attrib and "to" in child.attrib:
          #print("エッジ長とレーン番号の組",float(child2.attrib["length"]), lane_id)
          edge_length_dic[lane_id] = float(child2.attrib["length"])
          for i in range(len(node_x_list)):
            lane_dic[(x_y_dic[node_x_list[i],node_y_list[i]])] = lane_id
          lane_id += 1
          lane.set_others(float(child2.attrib["speed"]), node_id_list, node_x_list, node_y_list)
          edge_lanes_list.append(lane)  # to modify here

  return x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list

def create_road_segments(edge_lanes_list):
  road_segments_list = []
  for i in range(len(edge_lanes_list)-1):
    for j in range(i+1, len(edge_lanes_list)):
      if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
        road_segments_list.append(RoadSegment(edge_lanes_list[i], edge_lanes_list[j]))
        break
  return road_segments_list



def create_cars(number_of_cars, number_of_fake_cars, edges_cars_dic, DG):
    DG_copied = copy.deepcopy(DG)
    for i in range(len(obstacle_node_id_list)):
        DG_copied.remove_edge(pair_node_id_list[i], obstacle_node_id_list[i])
    for j in range(number_of_cars):
        origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()
        while True:
            try:
                shortest_path = nx.astar_path(DG_copied, origin_node_id, destination_node_id)
                break
            except Exception:
                origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()
        shortest_path = nx.dijkstra_path(DG, origin_node_id, destination_node_id)
        car = Car(origin_node_id, destination_node_id, destination_lane_id, shortest_path, origin_lane_id, DG)
        car.init(DG)
        cars_list.append(car)
        edges_cars_dic[(edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1])].append(car)
        if  opportunistic_communication_rate * number_of_cars < j:
            car.opportunistic_communication_frag = False

    for k in range(number_of_fake_cars):
        origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()
        while True:
            try:
                shortest_path = nx.astar_path(DG_copied, origin_node_id, destination_node_id)
                break
            except Exception:
                origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()

        shortest_path = nx.astar_path(DG, origin_node_id, destination_node_id)
        fakecar = fake_Car(origin_node_id, destination_node_id, destination_lane_id, shortest_path, origin_lane_id, DG)
        fakecar.init(DG)
        fakecar.create_fake_obstacle(obstacles_list, fake_obstacles_list, having_fake_obstacle)
        cars_list.append(fakecar)
        edges_cars_dic[(edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1])].append(fakecar)


def find_OD_node_and_lane():   #find_OD_node_and_lane()の定義

  origin_lane_id = np.random.randint(len(edge_lanes_list))  #開始地点Id
  destination_lane_id = np.random.randint(len(edge_lanes_list))  #　目的地id
  if origin_lane_id == destination_lane_id:                #開始位置と目的地が同じ場合の処理
    while origin_lane_id == destination_lane_id:
        destination_lane_id = np.random.randint(len(edge_lanes_list))  #edge_lanes_listの長さの範囲の整数の乱数を返す

  origin_node_id = x_y_dic[(edge_lanes_list[origin_lane_id].node_x_list[0], edge_lanes_list[origin_lane_id].node_y_list[0])]   #開始地点の辞書の作成？
  destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]  #終着地点の辞書の作成？

  while origin_node_id in obstacle_node_id_list:  #obstacle_node_id_listという辞書にorigin_node_idが含まれている間繰り返す
    origin_lane_id = np.random.randint(len(edge_lanes_list))  #edge_lanes_listの長さの範囲の整数の乱数を返す
    origin_node_id = x_y_dic[(edge_lanes_list[origin_lane_id].node_x_list[0], edge_lanes_list[origin_lane_id].node_y_list[0])]  #開始地点の辞書の作成？

  while destination_node_id in obstacle_node_id_list or origin_lane_id == destination_lane_id: #obstacle_node_id_listという辞書にdestination_node_idが含まれている、またはorigin_lane_id == destination_lane_idである間繰り返す
      destination_lane_id = np.random.randint(len(edge_lanes_list))   #edge_lanes_listの長さの範囲の整数の乱数を返す
      destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]  #終着地点の辞書の作成？

  return origin_lane_id, destination_lane_id, origin_node_id, destination_node_id
  
  
def find_obstacle_lane_and_node(edge_lanes_list, x_y_dic):    
        while True:
            obstacle_lane_id = np.random.randint(len(edge_lanes_list))
            obstacle_node_id = x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[-1], edge_lanes_list[obstacle_lane_id].node_y_list[-1])]
            oncoming_lane = None

            for i in range(len(edge_lanes_list) - 1):
                for j in range(i + 1, len(edge_lanes_list)):
                    if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
                        if edge_lanes_list[obstacle_lane_id] == edge_lanes_list[i]:
                            oncoming_lane = edge_lanes_list[j]
                        elif edge_lanes_list[obstacle_lane_id] == edge_lanes_list[j]:
                            oncoming_lane = edge_lanes_list[i]

            if oncoming_lane == None:
                if obstacle_node_id not in obstacle_node_id_list:
                    break
            elif oncoming_lane != None:
                if x_y_dic[(oncoming_lane.node_x_list[-1], oncoming_lane.node_y_list[-1])] not in obstacle_node_id_list and obstacle_node_id not in obstacle_node_id_list:
                    break

        obstacle_node_id_list.append(obstacle_node_id)
        pair_node_id_list.append(x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[0], edge_lanes_list[obstacle_lane_id].node_y_list[0])])

        return obstacle_lane_id, obstacle_node_id


def create_obstacles(number_of_obstacles, number_of_fake_cars, having_fake_obstacle, edge_lanes_list, x_y_dic, edges_obstacles_dic, edges_cars_dic, DG):
    while True:
        for total_obstacles in range(number_of_obstacles + number_of_fake_cars * having_fake_obstacle):
            obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node(edge_lanes_list, x_y_dic)
            #print(obstacle_lane_id)
            #print(obstacle_node_id)
            obstacle = Obstacle(obstacle_node_id, obstacle_lane_id)
            obstacle.init(DG)
            obstacles_list.append(obstacle)
            edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
            edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
        if nx.is_weakly_connected(DG) == True:
           break
 
 
#ネットワークの描画
def draw_road_network(DG):  
  pos=nx.get_node_attributes(DG,'pos')   
  edge_color = nx.get_edge_attributes(DG, "color")   
  nx.draw(DG, pos, node_size=1, arrowsize=5, with_labels=True, font_size=0.8, font_color="red", edge_color=edge_color.values())   



  
  

