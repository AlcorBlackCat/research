#!/usr/bin/env python3
# coding: utf-8

### import modules ###
from asyncore import read #from モジュール　import メソッドor変数 非同期で同時に複数動かすためのモジュール？　readが何なのかは不明。
from tkinter import X #GUIモジュール　xについては不明
import xml.etree.ElementTree as ET #xmlファイルを扱うためのモジュール
import sys #Pythonのインタプリタや実行環境に関する情報を扱うモジュール
import matplotlib.pyplot as plt #グラフ描画に関するモジュール
import networkx as nx #ネットワーク、グラフを生成する。恐らくmatplotlibと併用
import numpy as np #効率的な計算を行うためのモジュール、行列やベクトル向け？
from matplotlib.animation import FuncAnimation #複数のプロットを連続で表示してアニメーションを描画。シミュレーション中の描画と考えられる。
import math #数学計算用関数のモジュール
import copy #行列型等の数値において参照ではなくコピーを行うためのモジュール
import csv #csvファイルを管理するためのモジュール
from time import sleep #処理を一時停止させる関数

from grid_car import Car #詳細はgrid_car.py参照
from lane import Lane #lane.py
from grid_road_segment import RoadSegment #grid_road_segment.py
from obstacle import Obstacle #obstacle.py

### simulation settings ###
#infilename = "grid3x3.net.xml"
infilename = "grid5x5.net.xml"#別ファイルからマップ情報を引っ張ってきている
#infilename = "tsudanuma.net.xml"
#infilename = "sfc_small.net.xml"

#opportunistic_communication_frag = True
#a = np.random.randint(12345,123456)
a = 123456
#a = int(sys.argv[1])
print("seed値 : " + str(a))
np.random.seed(a)

#input parameters
number_of_cars = 300 #一般車両
number_of_obstacles = 10 #通行不能箇所数
number_of_fake_cars = 1 #悪意のある車両数
number_of_fake_obstacles = 1 #偽の通行不能箇所数
oppcomm_rate = 1.0  #
sensitivity = 1.0  #感度

math_count = 0 #数字カウント
avoid_count = 0 #通行不能個所？カウント

file_name = "result(" + str(a) + ") " + infilename + str(number_of_cars) + " " + str(number_of_obstacles) + " " + str(number_of_fake_cars) + " " + str(number_of_fake_obstacles) + ".csv"   #ファイル名を指定した値で連結し決定
folder_name = "result(csv)"    #folder_nameはresult(csv)に保存
if number_of_fake_cars >= 1:    #もし悪意のある車両が１以上ならば
 folder_name = "fake_result(csv)"    #folder_nameはフォルダー名fake_result(csv)に保存

folder_name2 = "moving_distance"    #folder_name2はmoving_distanceに保存
folder_name3 = "goal_time"    #folder_name2はgoal_timeに保存

print(number_of_cars,number_of_obstacles,number_of_fake_cars,number_of_fake_obstacles) #()内のものを出力
# functions
#xmlファイルを読み込み
def read_parse_netxml(infilename):  #resd_perse_netxml(引数名)で定義
  # open file
  infile = open(infilename, "r")  #infilnameを読み込み用でオープン

  # parsing xml 
  root = ET.fromstring(infile.read())  # XML を文字列から Element に直接パースします    ETはXML データを解析および作成するシンプルかつ効率的な API を実装するモジュール
  #print(root.tag, root.attrib)
  return root   #返り値をrootに格納　　　（rootを使うとこれらの処理の結果が用いられる）

#ネットワークの作成
def create_road_network(root):   #65行目で返したrootを引数に指定し、create_road_networkを定義
  # read edge tagged data for reading the road network
  # create data structure of road network using NetworkX
  x_y_dic = {} # input: node's x,y pos, output: node id  空の辞書の作成
  lane_dic = {}  #空の辞書の作成
  edge_length_dic = {}  #空の辞書の作成
  node_id = 0   #node_idの初期値を0に  node=各辺の交点？
  lane_id = 0   #lane_idの初期値0に   lane=道路?
 
  DG = nx.DiGraph()   # Directed graph of road network  有向グラフの場合の空のグラフ作成
  edge_lanes_list = []   # list of lane instances   空のリストの作成   辺、線のリスト
  for child in root:   #変数childにrootの要素が順に代入          #この辺のtagやattribはxmlファイルの読み込み関係
    if child.tag == "edge":  #もし、childのtagが"edge"に等しいならば
      lane = Lane()   #lane=クラスlane
      if "from" in child.attrib and "to" in child.attrib:   #もしfromとtoがchild.attribに含まれる場合
        lane.add_from_to(child.attrib["from"], child.attrib["to"])   #laneに要素を追加？

      for child2 in child:   #child2にchildの要素を順に代入、その後にそれぞれのリストを定義している
        data_list  = child2.attrib["shape"].split(" ")  
        node_id_list = []
        node_x_list = []; node_y_list = []
        distance_list = []
        data_counter = 0    #データの数の初期値

        for data in data_list:   #dataにdata_listの要素を順に代入
          node_x_list.append( float(data.split(",")[0]) )    #node_x_listにfloat型の数を追加　追加するのはdataの中から,で区切ってリストの中に追加した（.split）要素の０番目
          node_y_list.append( float(data.split(",")[1]) )    #node_y_listにfloat型の数を追加　追加するのはdataの中から,で区切ってリストの中に追加した（.split）要素の１番目
          if (float(data.split(",")[0]), float(data.split(",")[1])) not in x_y_dic.keys():   #もし、94,95行目のlistの中に x_y_dicの要素がなかった場合     辞書.keys()の形→キーのみが格納されたdict_keys型のイテラブル（要素を一つずつ取り出すことができるオブジェクト）を作る
            node_id_list.append(node_id)    #node_id_listにnode_idを追加
            DG.add_node(node_id, pos=(float(data.split(",")[0]), float(data.split(",")[1])))     #DG(グラフ？)のnodeに属性を付与 →　重み付きグラフを作成することができる
            x_y_dic[ (float(data.split(",")[0]), float(data.split(",")[1])) ] = node_id   #x_y_dicをnode_idに更新
            node_id += 1   #node_idを更新　カウントをする

          else:   #もし、94,95行目のlistの中に x_y_dicの要素があるなら
            node_id_list.append( x_y_dic[ (float(data.split(",")[0]), float(data.split(",")[1])) ] )   #node_id_listにx_y_dicを追加

          if data_counter >= 1:   #もし、data_counterが１以上なら
            distance_list.append( np.sqrt( (float(data.split(",")[0]) - old_node_x)**2 + (float(data.split(",")[1]) - old_node_y)**2) )   #指定された配列内の各要素の平方根を計算し、それをdistance_listに追加
          old_node_x = float(data.split(",")[0])    #old_node_xをfloat型で、dataの要素を,で区切ったときの０番目の要素にする
          old_node_y = float(data.split(",")[1])   #old_node_yをfloat型で、dataの要素を,で区切ったときの1番目の要素にする
          data_counter += 1   #data_counterを更新
        for i in range(len(node_id_list)-1):   #iにnode_id_listの長さ-1の範囲を順に代入
          DG.add_edge(node_id_list[i], node_id_list[i+1], weight=distance_list[i], color="black", speed=float(child2.attrib["speed"])) # calculate weight here  DGに辺を追加
        if "from" in child.attrib and "to" in child.attrib:  #もし、"from"がchild.attribの要素、"to"がchild.attribの要素なら
          #print("エッジ長とレーン番号の組",float(child2.attrib["length"]), lane_id)
          edge_length_dic[lane_id] = float(child2.attrib["length"])   #辺の長さの辞書？のlane_idを右辺のものとする
          for i in range(len(node_x_list)):   #iにnode_x_listの長さの範囲を順に代入
            lane_dic[(x_y_dic[node_x_list[i],node_y_list[i]])] = lane_id   #laneの辞書の[]内のものを右辺のものとする
          lane_id += 1   #lane_idを更新
          lane.set_others(float(child2.attrib["speed"]), node_id_list, node_x_list, node_y_list)   #計算や結果のために集合を作っている？  
          edge_lanes_list.append(lane)  # to modify here  edge_laneのリストにlaneを追加する

  return x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list    #この処理で得られた値をそれぞれ戻り値として返す

# generate a list of road segments for U-turn
#道路区分の作成
def create_road_segments(edge_lanes_list):   #関数定義
  road_segments_list = []   #空のリスト作成
  for i in range(len(edge_lanes_list)-1):   #iがeedge_lanes_list-1の長さ分だけ繰り返す
    for j in range(i+1, len(edge_lanes_list)):   #jがi+1～edge_lanes_listの長さの間の数だけ繰り返す
      if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:  #もし、この３つが等しいなら
        road_segments_list.append(RoadSegment(edge_lanes_list[i], edge_lanes_list[j]))   #road_segments_listに()内の要素を追加  RoadSegmentはclass
        break  #終了
  return road_segments_list  #返り値としてroad_segments_listを返す

# randomly select Orign and Destination lanes (O&D are different)
#出発地点と目的地をランダムに選ぶ
def find_OD_node_and_lane():   #find_OD_node_and_lane()の定義

  origin_lane_id = np.random.randint(len(edge_lanes_list))  #edge_lanes_listの長さの範囲の整数の乱数を返す
  destination_lane_id = origin_lane_id  # destination_lane_idをorigin_lane_idとする
  while origin_lane_id == destination_lane_id:  #origin_lane_id と destination_lane_idが等しい間繰り返す
    destination_lane_id = np.random.randint(len(edge_lanes_list))  #edge_lanes_listの長さの範囲の整数の乱数を返す

  origin_node_id = x_y_dic[(edge_lanes_list[origin_lane_id].node_x_list[0], edge_lanes_list[origin_lane_id].node_y_list[0])]   #開始地点の辞書の作成？
  destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]  #終着地点の辞書の作成？

  while origin_node_id in obstacle_node_id_list:  #obstacle_node_id_listという辞書にorigin_node_idが含まれている間繰り返す
    origin_lane_id = np.random.randint(len(edge_lanes_list))  #edge_lanes_listの長さの範囲の整数の乱数を返す
    origin_node_id = x_y_dic[(edge_lanes_list[origin_lane_id].node_x_list[0], edge_lanes_list[origin_lane_id].node_y_list[0])]  #開始地点の辞書の作成？

  while destination_node_id in obstacle_node_id_list or origin_lane_id == destination_lane_id: #obstacle_node_id_listという辞書にdestination_node_idが含まれている、またはorigin_lane_id == destination_lane_idである間繰り返す
      destination_lane_id = np.random.randint(len(edge_lanes_list))   #edge_lanes_listの長さの範囲の整数の乱数を返す
      destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]  #終着地点の辞書の作成？

  return origin_lane_id, destination_lane_id, origin_node_id, destination_node_id  #origin_lane_id, destination_lane_id, origin_node_id, destination_node_idを戻り値とする

#障害物を見つける
def find_obstacle_lane_and_node():   #関数の定義
  while True:   #Trueである間繰り返す
    obstacle_lane_id = np.random.randint(len(edge_lanes_list))  #edge_lanes_listの長さの範囲の整数の乱数を返す
    obstacle_node_id = x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[-1], edge_lanes_list[obstacle_lane_id].node_y_list[-1])]  #通行不能個所の辞書作成？
    oncoming_lane = None   #Noneは空っぽの状態を表すオブジェクト
    for i in range(len(edge_lanes_list) - 1):  #iがedge_lanes_listの長さの範囲-1だけ繰り返す
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
  #print("障害物ノードリスト : "+str(obstacle_node_id_list))
 
  return obstacle_lane_id, obstacle_node_id

#ネットワークの描画
def draw_road_network(DG):
  pos=nx.get_node_attributes(DG,'pos')
  edge_color = nx.get_edge_attributes(DG, "color")
  nx.draw(DG, pos, node_size=1, arrowsize=5, with_labels=True, font_size=0.8, font_color="red", edge_color=edge_color.values())

# For initializing animation settings
def init():
  line1.set_data([], [])
  line2.set_data([], [])
  line3.set_data([], [])
  line4.set_data([], [])
  title.set_text("Simulation step: 0")
  return line1, line2, line3, line4, title

# main of animation update
def animate(time):
  global xdata,ydata,obstacle_x,obstacle_y,Fxdata,Fydata,avoid_count,math_count,passing_comunication,goal_count
  global goal_time_list, number_of_shortest_path_changes_list, number_of_opportunistic_communication_list, moving_distance_list, time_list
  #sleep(0.1)

  xdata = []; ydata = []
  Fxdata = []; Fydata = []
  #all_cars_list = []
  #all_cars_list = cars_list + fakecars_list

  for car in cars_list:
    if car.__class__.__name__ == 'Car':
      time_list.append(time)
      x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)

      # remove arrived cars from the list
      if car.goal_arrived == True:
          goal_count += 1
          number_of_shortest_path_changes_list.append(car.number_of_shortest_path_changes)
          number_of_opportunistic_communication_list.append(car.number_of_opportunistic_communication)
          goal_time_list.append(car.elapsed_time)
          moving_distance_list.append(round(car.moving_distance,1))
          if car.fakecar_flag == False:
            #print("車両の削除")
            cars_list.remove( car )
          if car.fakecar_flag == True:
            cars_list.remove( car )
            print("悪意のある車の削除")
            #fakecars_list.remove( car )

      # TODO: if the car encounters road closure, it U-turns.
      #障害物があればUターン
      if car_forward_pt.__class__.__name__ != "Car" and diff_dist <= 20:
        if car_forward_pt.fake_flag == False:
          x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list)
        #偽の障害物ならmove
        else:
          None
          #print(car_forward_pt)
          #print(car)
        #print(car_forward_pt.fake_flag)
      
      xdata.append(x_new)
      ydata.append(y_new)
      if car.fakecar_flag == True:
        Fxdata.append(x_new)
        Fydata.append(y_new)
      #対向車線を決定 oc = oncoming = 対向
      #対向車線に車両があるとき、車両の持っている障害物の情報を渡す。

      if car.opportunistic_communication_frag == True: #すれ違い機能のON/OFF (35行目)
          for i in range(len(edge_lanes_list) - 1):
            for j in range(i + 1, len(edge_lanes_list)):
              if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
                if edge_lanes_list[car.current_lane_id] == edge_lanes_list[i]:
                  oc_lane = edge_lanes_list[j]
                elif edge_lanes_list[car.current_lane_id] == edge_lanes_list[j]:
                  oc_lane = edge_lanes_list[i]

          for oncoming_car in edges_cars_dic[(x_y_dic[(oc_lane.node_x_list[0], oc_lane.node_y_list[0])],x_y_dic[(oc_lane.node_x_list[1], oc_lane.node_y_list[1])])]:
            
            if oncoming_car.__class__.__name__ =="Car" and len(oncoming_car.obstacles_info_list) >= 1 and oncoming_car.opportunistic_communication_frag == True:
              #print("すれ違い通信の判定")
              #print(car.obstacles_info_list)
              for i in oncoming_car.obstacles_info_list:
                if i not in car.obstacles_info_list:
                  #print("すれ違い通信開始")
                  #car.number_of_opportunistic_communication += 1
                  car.obstacles_info_list.append(i)
                  if oncoming_car.fakecar_flag == False:
                    passing_comunication += 1 #相手は一般車両
                  else:
                    passing_comunication += 1 #相手は攻撃車両

                  if i in car.shortest_path:
                    for j in range(len(car.shortest_path)-1):
                      car.short_path.append((car.shortest_path[j],car.shortest_path[j+1]))
                    #print("経路" + str(car.short_path))
                    for j in car.short_path:
                      if j[1] == i:
                        #print(j)
                        if j[1] != car.shortest_path[car.current_sp_index + 1]:
                          if car.DG_copied.has_edge(j[0],j[1]) == True:
                            car.DG_copied.remove_edge(j[0],j[1])
                          while True:
                            try:
                              #print("-------------------")
                              #print(car)
                              #print("障害物" + str(i))
                              #print("旧最短経路" + str(car.shortest_path))
                              car.current_sp_index += 1
                              current_start_node_id = car.shortest_path[car.current_sp_index - 1]
                              #print("現在地:" + str(current_start_node_id) + " 目的地:" + str(car.dest_node_id))
                              car.shortest_path = nx.dijkstra_path(car.DG_copied, current_start_node_id, car.dest_node_id)
                              math_count += 1
                              #print("再計算" + str(math_count))
                              #print("新最短経路" + str(car.shortest_path))

                              car.current_sp_index = 0
                              current_start_node_id = car.shortest_path[car.current_sp_index]
                              car.current_start_node = car.DG_copied.nodes[current_start_node_id]["pos"]
                              car.current_position = car.DG_copied.nodes[current_start_node_id]["pos"]
                              current_end_node_id = car.shortest_path[car.current_sp_index + 1]
                              #print(current_start_node_id, current_end_node_id)
                              car.current_end_node = car.DG_copied.nodes[current_end_node_id]["pos"]
                              current_edge_attributes = car.DG_copied.get_edge_data(current_start_node_id, current_end_node_id)
                              car.current_max_speed = current_edge_attributes["speed"]
                              car.current_distance = current_edge_attributes["weight"]
                              edges_cars_dic[(current_start_node_id, current_end_node_id)].append(car)
                              
                              #print("-------------------")
                              break
                            except Exception:
                              #print(car)
                              car.dest_lane_id = np.random.randint(len(edge_lanes_list))
                              car.dest_node_id = x_y_dic[(edge_lanes_list[car.dest_lane_id].node_x_list[-1], edge_lanes_list[car.dest_lane_id].node_y_list[-1])]
                              while car.dest_node_id in obstacle_node_id_list or car.current_lane_id == car.dest_lane_id:
                                car.dest_lane_id = np.random.randint(len(edge_lanes_list))
                                car.dest_node_id = x_y_dic[(edge_lanes_list[car.dest_lane_id].node_x_list[-1], edge_lanes_list[car.dest_lane_id].node_y_list[-1])]
                              
                              avoid_count += 1

                        else:
                          x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list)
                          avoid_count += 1
                  
    #elif car.__class__.__name__ == 'Obstacle':
     # print("Obstacle #%d instance is called, skip!!!" % (car.obstacle_node_id))
    #elif car.__class__.__name__ == "Fire":


  obstacle_x = []; obstacle_y = []
  for obstacle in obstacles_list:
    x_new,y_new = obstacle.move()
    obstacle_x.append(x_new)
    obstacle_y.append(y_new)

  fakeobs_x = []; fakeobs_y = []
  for obstacle in fakeobs_list:
    x_new,y_new = obstacle.move()
    fakeobs_x.append(x_new)
    fakeobs_y.append(y_new)
  
  if time == 600:
    #print("残っている車両の確認")
    x = number_of_obstacles + number_of_fake_obstacles
    #print(cars_list)
    print(cars_list[x])
    print(cars_list[x].shortest_path)
    print("現在地" + str(cars_list[x].shortest_path[cars_list[x].current_sp_index]))
    print("強制終了")
    sys.exit(0)
  # check if all the cars arrive at their destinations
  if len(cars_list) - number_of_obstacles - number_of_fake_obstacles == 0:
    #print("経路変更回数"+str(number_of_shortest_path_changes_list))
    #print("すれ違い通信回数"+str(number_of_opportunistic_communication_list))
    #print("ゴールタイム"+str(goal_time_list))
    #print("総移動距離"+str(moving_distance_list))

    print("Total simulation step: " + str(time - 1))
    print("### End of simulation ###")
    print("remath:" + str(math_count) + " through:" + str(avoid_count) + " pass:" + str(passing_comunication))
    plt.clf()

    plt.hist(moving_distance_list, bins=50, rwidth=0.9, color='b')
    plt.xlabel("moving distance")
    plt.ylabel("number of cars")
    plt.savefig(folder_name2 + '/' + "総移動距離(" + str(a) + ") " + infilename + " " + str(number_of_cars) + " " + str(number_of_obstacles) + " " + str(number_of_fake_cars) + " " + str(number_of_fake_obstacles) + ".png")
    plt.clf()

    plt.hist(goal_time_list, bins=50, rwidth=0.9, color='b')
    plt.xlabel("goal time")
    plt.ylabel("number of cars")
    plt.savefig(folder_name3 + '/' +"ゴールタイム(" + str(a) + ") " + infilename + " " + str(oppcomm_rate) + " " + str(number_of_cars) + " " + str(number_of_obstacles) + " " + str(number_of_fake_cars) + " " + str(number_of_fake_obstacles) + ".png")
    plt.clf()

    """plt.hist(number_of_opportunistic_communication_list, bins=50,rwidth=0.9, color='b')
    # plt.show()
    plt.savefig("すれ違い数.png")
    plt.clf()

    plt.hist(number_of_shortest_path_changes_list, bins=50, rwidth=0.9,color='b')
    # plt.show()
    plt.savefig("経路変更数.png")
    plt.clf()"""

    with open(folder_name + '/' + file_name, 'w', newline='') as f:
      writer = csv.writer(f)
      for i in range(number_of_cars):
        writer.writerow([goal_time_list[i], moving_distance_list[i]])
    sys.exit(0) # end of simulation, exit.


  line1.set_data(xdata, ydata)
  line2.set_data(obstacle_x, obstacle_y)
  line3.set_data(Fxdata, Fydata)
  line4.set_data(fakeobs_x, fakeobs_y)
  title.set_text("Simulation step: " + str(time) + ";  # of cars: " + str(len(cars_list) - number_of_obstacles - number_of_fake_obstacles) + "; goal; " + str(goal_count))

  return line1, line2, line3, line4, title

##### main #####
if __name__ == "__main__":
  #print(opportunistic_communication_frag)
  # root: xml tree of input file
  root = read_parse_netxml(infilename)
  # x_y_dic: node's x,y pos --> node id
  # DG: Directed graph of road network
  # edge_lanes_list: list of lane instances
  x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list = create_road_network(root)
  #print(lane_dic)
  # road_segments_list: list of road segment instances
  road_segments_list = create_road_segments(edge_lanes_list)

  # create cars
  edges_all_list = DG.edges()
  edges_cars_dic = {}
  edges_obstacles_dic = {}

  for item in edges_all_list:
    edges_obstacles_dic[ item ] = []
    edges_cars_dic[ item ] = []
  #print(edges_cars_dic)


  obstacles_list = []
  fakeobs_list = []
  obstacle_node_id_list = []
  fakeobs_node_id_list = []
  pair_node_id_list = []
  cars_list = []
  fakecars_list = []
  obstacle_dic = {}

  goal_time_list = [] # 移動完了時間リスト
  number_of_shortest_path_changes_list = [] # 経路変更数リスト
  number_of_opportunistic_communication_list = [] # すれ違い通信数リスト
  moving_distance_list = []#総移動距離リスト
  time_list = []

  avoid_count = 0
  math_count = 0
  passing_comunication = 0
  goal_count = 0

  #edges_all_list = DG.edges()
  #create obstacles
  while True:
    for i in range(number_of_obstacles):
      obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node()
      obstacle = Obstacle(obstacle_node_id, obstacle_lane_id, False)
      obstacle.init(DG)
      obstacles_list.append(obstacle)
      cars_list.append(obstacle)
      edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
      edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
      obstacle_dic[edge_lanes_list[obstacle_lane_id].node_id_list[1]] = False
      #print(obstacle_dic)
    if nx.is_weakly_connected(DG) == True:
      break

  #偽の通行不能箇所(仮)
  while True:
    for i in range(number_of_fake_obstacles):
      obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node()
      obstacle = Obstacle(obstacle_node_id, obstacle_lane_id, True)
      obstacle.init(DG)
      fakeobs_list.append(obstacle)
      cars_list.append(obstacle)
      edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
      edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
      obstacle_dic[edge_lanes_list[obstacle_lane_id].node_id_list[1]] = True
      fakeobs_node_id_list.append(obstacle_node_id_list[number_of_obstacles + i])
    #print(obstacle_dic)
    #print(fakeobs_node_id_list)
    if nx.is_weakly_connected(DG) == True:
      break

  #車両作成
  DG_copied2 = copy.deepcopy(DG)
  for i in range(len(obstacle_node_id_list)):
    DG_copied2.remove_edge(pair_node_id_list[i],obstacle_node_id_list[i])
    #DG_copied2.remove_node(obstacle_node_id_list[i])
  for i in range(number_of_cars):
    # Reference: https://networkx.github.io/documentation/latest/reference/algorithms/generated/networkx.algorithms.shortest_paths.weighted.dijkstra_path.html
    origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()
    while True:
      try:
        shortest_path = nx.dijkstra_path(DG_copied2, origin_node_id, destination_node_id)
        break
      except Exception:
        origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()

    shortest_path = nx.dijkstra_path(DG, origin_node_id, destination_node_id)
    car = Car(origin_node_id, destination_node_id, destination_lane_id, shortest_path, origin_lane_id, DG, False)
    car.init(DG)  # initialization of car settings
    cars_list.append(car)
    edges_cars_dic[(edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1])].append(car)
    if oppcomm_rate * number_of_cars < i: #車両の割合ですれ違いのフラグのon/off
      car.opportunistic_communication_frag = False

  #悪意のある車両作成(仮)
  for j in range(number_of_fake_cars):
    origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()
    while True:
      try:
        shortest_path = nx.dijkstra_path(DG_copied2, origin_node_id, destination_node_id)
        break
      except Exception:
        origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()

    shortest_path = nx.dijkstra_path(DG, origin_node_id, destination_node_id)
    car = Car(origin_node_id, destination_node_id, destination_lane_id, shortest_path, origin_lane_id, DG, True)
    car.init(DG)  # initialization of car settings
    cars_list.append(car)
    fakecars_list.append(car)
    edges_cars_dic[(edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1])].append(car)
    for j in range(number_of_fake_obstacles):
      for k, v in obstacle_dic.items():
        if v == True and k not in car.obstacles_info_list:
          car.obstacle_dic[k] = True
          car.obstacles_info_list.append(k)
    #print("偽の通行不能箇所の辞書" + str(car.obstacle_dic))
    print(str(car) + "偽の通行不能箇所のリスト" + str(car.obstacles_info_list))
    if oppcomm_rate * number_of_fake_cars < j: #車両の割合ですれ違いのフラグのon/off
      car.opportunistic_communication_frag = False



  # animation initial settings
  fig, ax = plt.subplots()
  xdata = []; ydata = []
  for i in range(len(cars_list)):
    xdata.append( cars_list[i].current_position[0] )
    ydata.append( cars_list[i].current_position[1] )
  obstacle_x = []; obstacle_y = []
  for i in range(len(obstacles_list)):
    obstacle_x.append(obstacles_list[i].current_position[0])
    obstacle_y.append(obstacles_list[i].current_position[1])
  Fxdata = []; Fydata = []
  for i in range(len(fakecars_list)):
    Fxdata.append( fakecars_list[i].current_position[0] )
    Fydata.append( fakecars_list[i].current_position[1] )
  fakeobs_x = []; fakeobs_y = []
  for i in range(len(fakeobs_list)):
    fakeobs_x.append(fakeobs_list[i].current_position[0])
    fakeobs_y.append(fakeobs_list[i].current_position[1])


  line1, = plt.plot([], [], color="green", marker="s", linestyle="", markersize=5)
  line2, = plt.plot([], [], color="red", marker="s", linestyle="", markersize=5)
  line3, = plt.plot([], [], color="blue", marker="s", linestyle="", markersize=5)
  line4, = plt.plot([], [], color="cyan", marker="s", linestyle="", markersize=5)
  title = ax.text(20.0, -20.0, "", va="center")

  #img = Image.open(png_infilename)
  #img_list = np.asarray(img)
  #plt.imshow(img_list)
  ## draw road network
  draw_road_network(DG)


  print("通行不能箇所の辞書" + str(obstacle_dic))
  #print("remath:" + str(math_count) + " through:" + str(avoid_count) + " pass:" + str(passing_comunication))
  print("### Start of simulation ###")
  ani = FuncAnimation(fig, animate, frames=range(1000), init_func=init, blit=True, interval= 10)
  #ani.save("grid-sanimation.gif", writer='imagemagick')
  plt.show()
