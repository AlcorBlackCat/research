import networkx as nx
import numpy as np
import math
import copy
import pprint

class Car:
  def __init__(self, orig_node_id, dest_node_id, dest_lane_id, shortest_path, current_lane_id, DG):
    self.orig_node_id  = orig_node_id #起点
    self.dest_node_id  = dest_node_id #終点
    self.dest_lane_id = dest_lane_id
    self.shortest_path = shortest_path #最短経路
    self.current_lane_id =  current_lane_id #現在のレーン
    self.current_sp_index = 0
    self.current_speed = 0.0
    self.DG = DG
    self.current_start_node = []
    self.current_position = []
    self.current_end_node = []
    self.obstacles_info_list = []
    self.current_distance = 0.0
    self.number_of_shortest_path_changes = 0
    self.number_of_opportunistic_communication = 0
    self.elapsed_time = 0
    self.moving_distance = 0
    self.goal_arrived = False
    self.DG_copied = None
    self.opportunistic_communication_frag = True

  def init(self, DG):
    current_node_id = self.orig_node_id
    self.DG_copied = copy.deepcopy(DG)
    current_start_node_id = self.shortest_path[ self.current_sp_index ]
    self.current_start_node = DG.nodes[ current_start_node_id ]["pos"]
    self.current_position = DG.nodes[ current_start_node_id ]["pos"]
    current_end_node_id = self.shortest_path[ self.current_sp_index+1]
    self.current_end_node = DG.nodes[ current_end_node_id ]["pos"]
    current_edge_attributes = DG.get_edge_data(current_start_node_id, current_end_node_id)
    self.current_max_speed = current_edge_attributes["speed"]
    self.current_distance = current_edge_attributes["weight"]

  # Optimal Velocity Function to determine the current speed
  def V(self, inter_car_distance):
    return 0.5*self.current_max_speed*(np.tanh(inter_car_distance-2) + np.tanh(2))

  # update car's speed
  #inter_car_distance = diff_dist
  def update_current_speed(self, sensitivity, inter_car_distance):
    self.current_speed += sensitivity*( self.V(inter_car_distance) - self.current_speed )

  def move(self, edges_cars_dic, sensitivity, lane_dic, edge_length_dic):

    self.elapsed_time += 1
    #self.moving_distance += edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]]
    #print(edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]])
    # x_prev == self.current_position[0]
    # y_prev == self.current_position[1]
    direction_x = self.current_end_node[0] - self.current_position[0]
    direction_y = self.current_end_node[1] - self.current_position[1]
    arg = math.atan2(direction_y, direction_x)

    arrived_cars_list = []

    #x_new = None; y_new = None
    #print(edge_length_dic.get(lane_dic[self.shortest_path[self.current_sp_index]]))
    #pprint.pprint(lane_dic)
    #print(self.shortest_path)


    #print(self.moving_distance)
    #edge_length_dic  key : lane_id, value : edge_length
    if np.sqrt((self.current_position[0] - self.current_end_node[0])**2 + (self.current_position[1] - self.current_end_node[1])**2) < self.current_speed: # to arrive at the terminal of edge
      if self.shortest_path[self.current_sp_index] in lane_dic:
        self.moving_distance += edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]]
      self.current_sp_index += 1
      if self.current_sp_index >= len(self.shortest_path)-1: # arrived at the goal 
        self.goal_arrived = True
        x_new = self.current_end_node[0]
        y_new = self.current_end_node[1]

        current_start_node_id = self.shortest_path[ self.current_sp_index-1 ]
        current_end_node_id = self.shortest_path[ self.current_sp_index ]
        self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]

        car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
        car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
        diff_dist = 50.0

        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].remove( self )
        arrived_cars_list.append( self )

      else: # lane change
        if self.shortest_path[self.current_sp_index] in lane_dic:
          self.moving_distance += edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]]
        x_new = self.current_end_node[0]
        y_new = self.current_end_node[1]

        current_start_node_id = self.shortest_path[ self.current_sp_index-1 ]
        current_end_node_id = self.shortest_path[ self.current_sp_index ]
        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].remove( self )

        current_start_node_id = self.shortest_path[ self.current_sp_index ]
        self.current_start_node = self.DG_copied.nodes[ current_start_node_id ]["pos"]
        self.current_position = self.DG_copied.nodes[ current_start_node_id ]["pos"]
        current_end_node_id = self.shortest_path[ self.current_sp_index+1]
        self.current_end_node = self.DG_copied.nodes[ current_end_node_id ]["pos"]
        current_edge_attributes = self.DG_copied.get_edge_data(current_start_node_id, current_end_node_id)
        #self.current_max_speed = current_edge_attributes["speed"]
        #self.current_distance = current_edge_attributes["weight"]
        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].append( self )
        #print(edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)) #レーン内の車両または障害物の数

        if edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) > 0:
          car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) - 1
          car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
          diff_dist = 50

        else:
          car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
          car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
          diff_dist = 50.0


    else: # move to the terminal of edge
      x_new = self.current_position[0] + self.current_speed*np.cos(arg)
      y_new = self.current_position[1] + self.current_speed*np.sin(arg)
      self.current_position = [x_new, y_new]
      current_start_node_id = self.shortest_path[ self.current_sp_index ]
      current_end_node_id = self.shortest_path[ self.current_sp_index+1 ]

      if edges_cars_dic[ (current_start_node_id, current_end_node_id) ].index( self ) > 0:
        car_forward_index = edges_cars_dic[ (current_start_node_id, current_end_node_id) ].index( self ) - 1
        car_forward_pt = edges_cars_dic[ (current_start_node_id, current_end_node_id) ][ car_forward_index ]
        diff_dist = np.sqrt( (car_forward_pt.current_position[0] - self.current_position[0])**2 + (car_forward_pt.current_position[1] - self.current_position[1])**2 )

      else:
        car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
        car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
        diff_dist = 50.0
      self.update_current_speed(sensitivity, diff_dist)
      self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]
    return x_new, y_new, car_forward_pt, diff_dist

  def U_turn(self, edges_cars_dic,lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list):
    self.current_sp_index += 1


    x_new = self.current_end_node[0]
    y_new = self.current_end_node[1]

    # Uターン前のモデルの削除
    current_start_node_id = self.shortest_path[self.current_sp_index - 1]
    current_end_node_id = self.shortest_path[self.current_sp_index]
    edges_cars_dic[(current_start_node_id, current_end_node_id)].remove(self)
    pre_start_node_id = current_start_node_id
    pre_end_node_id = current_end_node_id


    #発見した障害物ノードidを保存
    if current_end_node_id not in self.obstacles_info_list:
      self.obstacles_info_list.append(current_end_node_id)

    self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]
    #print("今のノードの番号:"+str(self.shortest_path[self.current_sp_index]))
    #print("今いるレーンの番号:"+str(self.current_lane_id))

    for i in range(len(edge_lanes_list) - 1):
      for j in range(i + 1, len(edge_lanes_list)):
        if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
          if edge_lanes_list[self.current_lane_id] == edge_lanes_list[i]:
            #print("対向車線のレーンの番号"+str(j))
            #print("車線変更後のノード番号"+str(x_y_dic[(edge_lanes_list[j].node_x_list[0], edge_lanes_list[j].node_y_list[0])]))
            current_start_node_id = x_y_dic[(edge_lanes_list[j].node_x_list[0], edge_lanes_list[j].node_y_list[0])]
            current_end_node_id = x_y_dic[(edge_lanes_list[j].node_x_list[-1], edge_lanes_list[j].node_y_list[-1])]

          elif edge_lanes_list[self.current_lane_id] == edge_lanes_list[j]:
            #print("車線変更後のレーンの番号"+str(i))
            #print("車線変更後のノード番号"+str(x_y_dic[(edge_lanes_list[i].node_x_list[0], edge_lanes_list[i].node_y_list[0])]))
            current_start_node_id = x_y_dic[(edge_lanes_list[i].node_x_list[0], edge_lanes_list[i].node_y_list[0])]
            current_end_node_id = x_y_dic[(edge_lanes_list[i].node_x_list[-1], edge_lanes_list[i].node_y_list[-1])]

    self.current_start_node = self.DG_copied.nodes[current_start_node_id]["pos"]
    self.current_position = self.DG_copied.nodes[current_start_node_id]["pos"]
    self.current_end_node = self.DG_copied.nodes[current_end_node_id]["pos"]
    #print(x_y_dic[self.current_position])#ノード番号

    #障害物を含むedgeの削除
    if self.DG_copied.has_edge(pre_start_node_id, pre_end_node_id) == True:
      self.DG_copied.remove_edge(pre_start_node_id, pre_end_node_id)

    #最短経路の再計算
    #print("車線変更前のshortest_path"+str(self.shortest_path))
    while True:
      try:
        self.shortest_path = nx.dijkstra_path(self.DG_copied, current_start_node_id, self.dest_node_id)
        break
      except Exception:
          self.dest_lane_id = np.random.randint(len(edge_lanes_list))
          self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]
          while self.dest_node_id in obstacle_node_id_list or self.current_lane_id == self.dest_lane_id:
            self.dest_lane_id = np.random.randint(len(edge_lanes_list))
            self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]

    #print("車線変更後のshortest_path" + str(self.shortest_path))
    self.number_of_shortest_path_changes += 1
    self.current_sp_index = 0# current_sp_indexのリセット

    current_start_node_id = self.shortest_path[self.current_sp_index]
    self.current_start_node = self.DG_copied.nodes[current_start_node_id]["pos"]
    self.current_position = self.DG_copied.nodes[current_start_node_id]["pos"]
    current_end_node_id = self.shortest_path[self.current_sp_index + 1]
    self.current_end_node = self.DG_copied.nodes[current_end_node_id]["pos"]
    current_edge_attributes = self.DG_copied.get_edge_data(current_start_node_id, current_end_node_id)
    self.current_max_speed = current_edge_attributes["speed"]
    self.current_distance = current_edge_attributes["weight"]
    edges_cars_dic[(current_start_node_id, current_end_node_id)].append(self)
   # print('U_turn end!')

    return x_new, y_new


class FakeCar:
    def __init__(self, orig_node_id, dest_node_id, dest_lane_id, shortest_path, orig_lane_id):
        self.orig_node_id = orig_node_id
        self.dest_node_id = dest_node_id
        self.dest_lane_id = dest_lane_id
        self.shortest_path = shortest_path
        self.current_lane_id = orig_lane_id
        self.current_position = DG.nodes[orig_node_id]["pos"]
        self.speed = 0.0
        self.is_fake_obstacle_ahead_flag = False
        self.goal_arrived = False

        # グラフにFakeCarオブジェクトを追加する
        DG.add_node(self, obj=self)

    def move(self, edges_cars_dic, sensitivity, lane_dic, edge_length_dic):
        # Calculate the next position of the fake car.
        next_node = self.current_lane.to_id
        x_new, y_new = self.calc_next_position(self.current_position, next_node, self.velocity, sensitivity)

        # Update the current position of the fake car.
        self.current_position = (x_new, y_new)

        # Add the FakeCar object to DG graph nodes.
        self.DG.nodes[next_node]["obj"] = self

        # Return the new position of the fake car and other information.
        return x_new, y_new, None, None, None

    def is_fake_obstacle_ahead(self):
        fake_obstacle_ahead = False

        # 車両の経路が存在し、次に到達するノードが偽の通行不能箇所かどうかを判定
        if self.shortest_path and self.current_sp_index < len(self.shortest_path) - 1:
            # 次に到達するノード
            next_node = self.shortest_path[self.current_sp_index + 1]

            # 次のノードがFakeObstacleであるかどうかを判定
            fake_obstacle_ahead = isinstance(self.DG.nodes[next_node]["obj"], FakeObstacle)

        return fake_obstacle_ahead
