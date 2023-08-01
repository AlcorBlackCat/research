#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#関数を以下に集約
#目次
#8 グローバル変数

number_of_cars = 300 
number_of_obstacles = 10 
number_of_fake_cars = 1 
number_of_fake_obstacles = 1 
opportunistic_communication_rate = 1.0  
sensitivity = 1.0

obstacles_list = []
fakeobs_list = []
obstacle_node_id_list = []
fakeobs_node_id_list = []
pair_node_id_list = []
fakepair_node_id_list = []
cars_list = []
fakecars_list = []
obstacle_dic = {}

goal_time_list = []
number_of_shortest_path_changes_list = []
number_of_opportunistic_communication_list = []
moving_distance_list = []
time_list = []

def find_OD_node_and_lane():   #find_OD_node_and_lane()の定義

  origin_lane_id = np.random.randint(len(edge_lanes_list))  #開始地点Id
  destination_lane_id = np.random.randint(len(edge_lane_list))  #　目的地id
  if origin_lane_id == destination_lane_id:                #開始位置と目的地が同じ場合の処理
    while origin_lane_id == destinat_lane_id:
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
  
  
def find_obstacle_lane_and_node():   #関数の定義
  for _ in range(number_of_obstacles):  # 指定した回数だけ繰り返し処理を行うfor 文を追加
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

    return obstacle_lane_id, obstacle_node_id, obstacle_node_id_list, pair_node_id_list   #戻り値にobstacle_node_id_list, pair_node_id_listの追加
 
 
#ネットワークの描画
def draw_road_network(DG):  #draw_road_networkという関数の定義  引数はDG
  pos=nx.get_node_attributes(DG,'pos')   #get_node_attributesはグラフからノード属性を取得する　　DGのグラフからposという属性を取り出す？
  edge_color = nx.get_edge_attributes(DG, "color")   #get_edge_attributesはグラフからedge属性を取得する  DGのグラフからcolorという属性を取り出す
  nx.draw(DG, pos, node_size=1, arrowsize=5, with_labels=True, font_size=0.8, font_color="red", edge_color=edge_color.values())   #ネットワークの可視化


#  For initializing animation settings   (アニメーション設定を初期化する場合)
def init():
  line1.set_data([], [])  #グラフにプロットする普通車の初期化
  line2.set_data([], [])  #通行不能箇所
  line3.set_data([], [])  #悪意を持った車両
  line4.set_data([], [])  #偽の通行不能箇所
  title.set_text("Simulation step: 0")   #titeleに（）内の文字をセット？
  return line1, line2, line3, line4, title
  
  

