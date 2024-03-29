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
number_of_cars = 300 #一般車両、奇数にしない方が良さそう？
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
  for child in root:   #変数childにrootの要素が順に代入          #この辺のtagやattribはxmlファイルの読み込み関係  https://qiita.com/sino20023/items/0314438d397240e56576
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
  destination_lane_id = origin_lane_id  # destination_lane_idをorigin_lane_idとする　目的地id
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

#障害物を見つける  ここが障害物情報を保管するコード
def find_obstacle_lane_and_node():   #関数の定義
  while True:   #Trueである間繰り返す
    obstacle_lane_id = np.random.randint(len(edge_lanes_list))  #edge_lanes_listの長さの範囲の整数の乱数を返す
    obstacle_node_id = x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[-1], edge_lanes_list[obstacle_lane_id].node_y_list[-1])]  #通行不能個所の辞書作成？
    oncoming_lane = None   #Noneは空っぽの状態を表すオブジェクト
    for i in range(len(edge_lanes_list) - 1):
      for j in range(i + 1, len(edge_lanes_list)):   #i+1～edge_lanes_listの長さの範囲
        if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:   #左式の等号と右式の等号がおなじなら?
          if edge_lanes_list[obstacle_lane_id] == edge_lanes_list[i]:  #edge_lanes_listが保持している通行不能個所情報とedge_lanes_listのiの要素が等しいなら
            oncoming_lane = edge_lanes_list[j]   #oncoming_lane（対向レーン）はedge_lanes_list[j]
          elif edge_lanes_list[obstacle_lane_id] == edge_lanes_list[j]: 
            oncoming_lane = edge_lanes_list[i]
    if oncoming_lane == None:   #もしoncoming_laneが空なら
      if obstacle_node_id not in obstacle_node_id_list:   #もしobstacle_node_idがobstacle_node_id_listという辞書に無かった場合
        break
    elif oncoming_lane != None:  #oncoming_laneが空ではないとき
      if x_y_dic[(oncoming_lane.node_x_list[-1], oncoming_lane.node_y_list[-1])] not in obstacle_node_id_list and obstacle_node_id not in obstacle_node_id_list:   #x_y_dic(道路の座標情報？)がobstacle_node_id_listに含まれなく、obstacle_node_idがobstacle_node_id_listに含まれない場合
        break
  obstacle_node_id_list.append(obstacle_node_id)   #obstacle_node_id_listに（）内の要素の追加
  pair_node_id_list.append(x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[0], edge_lanes_list[obstacle_lane_id].node_y_list[0])])   #辞書かリストに要素追加  おそらく、対向車線も含めた道路情報
  #print("障害物ノードリスト : "+str(obstacle_node_id_list))     obstacle_node_id_listが通行不能個所の一覧になっている
 
  return obstacle_lane_id, obstacle_node_id

"""
def find_fakeobs_lane_and_node():
  while True:
    fakeobs_lane_id = np.random.randint(len(edge_lanes_list))
    fakeobs_node_id = x_y_dic[(edge_lanes_list[fakeobs_lane_id].node_x_list[-1], edge_lanes_list[fakeobs_lane_id].node_y_list[-1])]
    oncoming_lane = None
    for i in range(len(edge_lanes_list) - 1):
      for j in range(i + 1, len(edge_lanes_list)):   
        if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:   
          if edge_lanes_list[fakeobs_lane_id] == edge_lanes_list[i]:  
            oncoming_lane = edge_lanes_list[j]   
          elif edge_lanes_list[fakeobs_lane_id] == edge_lanes_list[j]: 
            oncoming_lane = edge_lanes_list[i]
    if oncoming_lane == None:   
      if fakeobs_node_id not in fakeobs_node_id_list:   
        break
    elif oncoming_lane != None:  
      if x_y_dic[(oncoming_lane.node_x_list[-1], oncoming_lane.node_y_list[-1])] not in obstacle_node_id_list and fakeobs_node_id not in obstacle_node_id_list:   
        break
  fakeobs_node_id_list.append(obstacle_node_id)   
  fakepair_node_id_list.append(x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[0], edge_lanes_list[obstacle_lane_id].node_y_list[0])])

  return fakeobs_lane_id, fakeobs_node_id
"""
  
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

# main of animation update   (アニメーションアップデートのメイン)
def animate(time):
  global xdata,ydata,obstacle_x,obstacle_y,Fxdata,Fydata,avoid_count,math_count,passing_comunication,goal_count   #グローバル変数の定義 グローバル変数はモジュール(スクリプト)全体で有効な変数です
  global goal_time_list, number_of_shortest_path_changes_list, number_of_opportunistic_communication_list, moving_distance_list, time_list
  #sleep(0.1)

  xdata = []; ydata = []  #それぞれの変数の初期化
  Fxdata = []; Fydata = []
  #all_cars_list = []
  #all_cars_list = cars_list + fakecars_list

  for car in cars_list:  #carはcars_listの回数分繰り返す
    if car.__class__.__name__ == 'Car':  #もしcarの_class_の_name_が'Car'に等しいなら？  ''は""と同じ　　Car→grid_car　　??
      time_list.append(time)   #time_listに(time)を追加  時間情報を保持しておくための入れ物？
      x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)   #挿し木のグローバル変数の定義？？？

      # remove arrived cars from the list   到着した車をリストから削除します
      if car.goal_arrived == True:  #もしcarのgoal_arrivedが Trueなら   goal到着しているなら
          goal_count += 1   #goal_countを更新
          number_of_shortest_path_changes_list.append(car.number_of_shortest_path_changes)  #最短経路に変更した数のリストにcarが持っている最短経路の変更数を追加？
          number_of_opportunistic_communication_list.append(car.number_of_opportunistic_communication)  #すれ違い通信をした数のリストにcarがもっているすれ違い通信数を追加
          goal_time_list.append(car.elapsed_time)  #goal_time_listにcarがもっているelapsed_time(経過時間情報？)を追加
          moving_distance_list.append(round(car.moving_distance,1))  #moving_distance_lisにcarのmoving_distanceを小数点以下１桁を表示するように四捨五入したものを追加
          if car.fakecar_flag == False:   #もしcarのfakecar_flagがFalseのとき
            #print("車両の削除")
            cars_list.remove( car )  #cars_listから車両の削除
          if car.fakecar_flag == True:  #もしcarのfakecar_flagがTrueのとき
            cars_list.remove( car )  #cars_listから車両の削除  
            print("悪意のある車の削除")  
            #fakecars_list.remove( car )

      # TODO: if the car encounters road closure, it U-turns.    (車が通行止めに遭遇した場合、それはUターンします)     アノテーションコメント→コメントにメタデータを付加するものです。コードの欠陥がわかりやすくなります
      #障害物があればUターン
      if car_forward_pt.__class__.__name__ != "Car" and diff_dist <= 20:
        if car_forward_pt.fake_flag == False:    #もし、car_forward_ptのfake_flagがFalseなら
          x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list)  #x_new とy_newを更新
        #偽の障害物ならmove
        else:
          None
          #print(car_forward_pt)
          #print(car)
        #print(car_forward_pt.fake_flag)
      
      xdata.append(x_new)   #グローバル変数のxdataにx_newを追加
      ydata.append(y_new)   #グローバル変数のydataにy_newを追加
      if car.fakecar_flag == True:   #もし、carのfake_flagがTrueなら
        Fxdata.append(x_new)  #たぶんこのFはfakeのF　悪意を持った車両用のデータ？　　xdataにx_newを追加
        Fydata.append(y_new)  #xdataにx_newを追加
      #対向車線を決定 oc = oncoming = 対向
      #対向車線に車両があるとき、車両の持っている障害物の情報を渡す。

      if car.opportunistic_communication_frag == True: #すれ違い機能のON/OFF (35行目)
          for i in range(len(edge_lanes_list) - 1):
            for j in range(i + 1, len(edge_lanes_list)):
              if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:  #edge_lane_id[i]のfrom_idとedge_lane_id[j]のto_idが同じでなおかつ、edge_lane_id[i]のto_idとedge_lane_id[j]のfrom_idが等しいとき　　おそらく、すべてのノードの組み合わせを網羅的に比べていき、行くときも来るときも同じノードである場合のみと押す
                if edge_lanes_list[car.current_lane_id] == edge_lanes_list[i]:  #現在のlane_idとiが等しいなら
                  oc_lane = edge_lanes_list[j]  #対向車線の更新
                elif edge_lanes_list[car.current_lane_id] == edge_lanes_list[j]:  #
                  oc_lane = edge_lanes_list[i]  #対向車線の更新

          for oncoming_car in edges_cars_dic[(x_y_dic[(oc_lane.node_x_list[0], oc_lane.node_y_list[0])],x_y_dic[(oc_lane.node_x_list[1], oc_lane.node_y_list[1])])]:  #oncoming_car(対向車線の車)をedges_cars_dicの数だけ繰り返す
            
            if oncoming_car.__class__.__name__ =="Car" and len(oncoming_car.obstacles_info_list) >= 1 and oncoming_car.opportunistic_communication_frag == True:   # oncoming_car.__class__.__name__ =="Car"でかつ、oncoming_car.obstacles_info_list（対向車線の車が持っている通行不能個所の情報）が１以上かつ、oncoming_car.opportunistic_communication_flag ==True（すれ違い通信をする）とき
              #print("すれ違い通信の判定")
              #print(car.obstacles_info_list)
              for i in oncoming_car.obstacles_info_list:  #対向車線の車が持つ通行不能個所の情報だけ繰り返す
                if i not in car.obstacles_info_list:  #iがcar.obstacles_info_list（車がもつ通行不能個所情報（自分？））にない場合
                  #print("すれ違い通信開始")
                  #car.number_of_opportunistic_communication += 1
                  car.obstacles_info_list.append(i)   #car.obstacles_info_listにi(新しい通行不能個所の情報)を追加
                  if oncoming_car.fakecar_flag == False:  #対向車線の車のfakecar_flagがFalse(悪意を持つ車両ではない)とき
                    passing_comunication += 1 #相手は一般車両
                  else:
                    passing_comunication += 1 #相手は攻撃車両

                  if i in car.shortest_path:  #iが最短経路の数だけ繰り返す
                    for j in range(len(car.shortest_path)-1):  
                      car.short_path.append((car.shortest_path[j],car.shortest_path[j+1]))  #carの最短経路に追加
                    #print("経路" + str(car.short_path))
                    for j in car.short_path:  #jが最短経路の数だけ繰り返す
                      if j[1] == i:  #jの１つめの要素がiと等しいなら
                        #print(j)
                        if j[1] != car.shortest_path[car.current_sp_index + 1]:  #jの１つめの要素がcarの最短経路の[]内の要素と等しくないなら
                          if car.DG_copied.has_edge(j[0],j[1]) == True:   #もしDDグラフのコピー？のedge[]がTrueなら？？？
                            car.DG_copied.remove_edge(j[0],j[1])  #edgeの削除
                          while True:  #Trueである間繰り返す
                            try:
                              #print("-------------------")
                              #print(car)
                              #print("障害物" + str(i))
                              #print("旧最短経路" + str(car.shortest_path))
                              car.current_sp_index += 1  #カウント？値？の更新
                              current_start_node_id = car.shortest_path[car.current_sp_index - 1]  #現在の開始地点を最短経路に更新？？？     shortest_path()はそれで一つの関数
                              #print("現在地:" + str(current_start_node_id) + " 目的地:" + str(car.dest_node_id))
                              car.shortest_path = nx.dijkstra_path(car.DG_copied, current_start_node_id, car.dest_node_id)  #ダイクストラ法（Dijkstra's algorithm）は辺の重みが非負数のグラフの単一始点最短経路問題を解くアルゴリズム。　dest = 目的地
                              math_count += 1  #カウントの更新
                              #print("再計算" + str(math_count))
                              #print("新最短経路" + str(car.shortest_path))

                              car.current_sp_index = 0  #初期値更新？
                              current_start_node_id = car.shortest_path[car.current_sp_index]    #shortest_pathは全点対最短経路問題に対する解が返される。各要素の値が最短経路のコストの総和（最短距離）
                              car.current_start_node = car.DG_copied.nodes[current_start_node_id]["pos"]   #開始地点のnodeをcarのDGのcurrent_start_node_idから"pos"という属性でコピーする？
                              car.current_position = car.DG_copied.nodes[current_start_node_id]["pos"]  #nodeのコピー　ポジション
                              current_end_node_id = car.shortest_path[car.current_sp_index + 1]  #終了地点？ゴール地点を  保存されていた？最短経路から更新する
                              print(current_start_node_id, current_end_node_id)  #ちゃんとできているか確認用
                              car.current_end_node = car.DG_copied.nodes[current_end_node_id]["pos"]  #車が保持している？終了地点をDGのnodeのcurrent_end_node_idという要素からコピー？
                              current_edge_attributes = car.DG_copied.get_edge_data(current_start_node_id, current_end_node_id)  #現在の終了地点　　get_edge_data(u, v, デフォルト = なし)[ソース]     エッジ (u, v) に関連付けられた属性ディクショナリを返します  
                              car.current_max_speed = current_edge_attributes["speed"]  #carの現在の最大速度？ = edgeの"speed"という属性に更新
                              car.current_distance = current_edge_attributes["weight"]  #carの現在の距離 = edgeの"weight"という属性に更新  辺の重み
                              edges_cars_dic[(current_start_node_id, current_end_node_id)].append(car)  #edges_cars_dicの[]ないのリストにcarを追加？
                              
                              #print("-------------------")
                              break
                            except Exception:  #例外処理　　Exceptionという例外
                              #print(car)
                              car.dest_lane_id = np.random.randint(len(edge_lanes_list))  #()内の範囲の整数の乱数を返す
                              car.dest_node_id = x_y_dic[(edge_lanes_list[car.dest_lane_id].node_x_list[-1], edge_lanes_list[car.dest_lane_id].node_y_list[-1])]  #x_y_dicのそれぞれnodeのx軸y軸情報に更新  -1はなに？
                              while car.dest_node_id in obstacle_node_id_list or car.current_lane_id == car.dest_lane_id:  #obstacle_node_id_listにcar.dest_node_idが含まれる、またはcar.current_lane_idがcar.dest_lane_idである間繰り返す
                                car.dest_lane_id = np.random.randint(len(edge_lanes_list))  #()内の範囲の整数の乱数を返し更新する
                                car.dest_node_id = x_y_dic[(edge_lanes_list[car.dest_lane_id].node_x_list[-1], edge_lanes_list[car.dest_lane_id].node_y_list[-1])]  #x_y_dicのそれぞれnodeのx軸y軸情報に更新　　-1はおそらくdestの時に返した値が１～で本来のものより1つずれている？から-1をする？
                              
                              avoid_count += 1  #カウントの更新

                        else:  #ifにたいしての例外処理
                          x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list)  #あらたなx軸y軸情報に更新？更新するものはUターン情報から？
                          avoid_count += 1  #カウントの更新  グローバル変数
                  
    #elif car.__class__.__name__ == 'Obstacle':
     # print("Obstacle #%d instance is called, skip!!!" % (car.obstacle_node_id))
    #elif car.__class__.__name__ == "Fire":


  obstacle_x = []; obstacle_y = []  #通行不能のx軸のリスト,y軸のリストの初期化
  for obstacle in obstacles_list:  #obstacles_listから順に取り出し処理したものをobstacleとする
    x_new,y_new = obstacle.move()  #moveは、図形を指定したピクセル分の距離の場所に移動？？
    obstacle_x.append(x_new)  #obstacle_xにx_newを追加
    obstacle_y.append(y_new)  #obstacle_yにy_newを追加  y軸情報のリストにｙの位置情報を更新？

  fakeobs_x = []; fakeobs_y = []  #偽の通行不能個所のx軸のリスト,y軸のリストを初期化
  for obstacle in fakeobs_list:  #fakeobs_listから順に取り出し処理したものをobstacleとする
    x_new,y_new = obstacle.move()  #moveは、図形を指定したピクセル分の距離の場所に移動？？
    fakeobs_x.append(x_new)  #fakeobs_xにx_newを追加
    fakeobs_y.append(y_new)  #fakeobs_ｙにy_newの追加
  
  if time == 600:  #もし、timeが600なら　何かしらの不具合でシミュレーションが終わらなかった場合の処理？
    #print("残っている車両の確認")
    x = number_of_obstacles + number_of_fake_obstacles  #x = 通行不能個所数 + 偽の通行不能個所数
    #print(cars_list)
    print(cars_list[x])  #cars_listの合計通行不能個所数？を表示
    print(cars_list[x].shortest_path)  #cars_listの最短経路の合計通行不能個所数？の表示
    print("現在地" + str(cars_list[x].shortest_path[cars_list[x].current_sp_index]))  #現在地の座標を表示する  sp=shortest_path？？
    print("強制終了")
    sys.exit(0)  #実行しているプログラムの中でメインプロセスを終了させるための関数
  # check if all the cars arrive at their destinations
  if len(cars_list) - number_of_obstacles - number_of_fake_obstacles == 0:  #もしcars_listの長さ - 通行不能個所数 - 偽の通行不能個所数 =0ならば
    #print("経路変更回数"+str(number_of_shortest_path_changes_list))
    #print("すれ違い通信回数"+str(number_of_opportunistic_communication_list))
    #print("ゴールタイム"+str(goal_time_list))
    #print("総移動距離"+str(moving_distance_list))

    print("Total simulation step: " + str(time - 1))  #シミュレーションのかかった合計step数を表示
    print("### End of simulation ###")  
    print("remath:" + str(math_count) + " through:" + str(avoid_count) + " pass:" + str(passing_comunication))  #再計算？カウント + 通行カウント？？ + 通信で渡したカウント？ を表示
    plt.clf()  #Clear figure    現在のFigure全体とそのすべての軸をクリアしますが、ウィンドウを開いたままにして、他のプロットで再利用できるようにします。

    plt.hist(moving_distance_list, bins=50, rwidth=0.9, color='b')  #移動距離のヒストグラム（正規化はされていない）を描く
    plt.xlabel("moving distance")  #軸にラベルを付ける
    plt.ylabel("number of cars")
    plt.savefig(folder_name2 + '/' + "総移動距離(" + str(a) + ") " + infilename + " " + str(number_of_cars) + " " + str(number_of_obstacles) + " " + str(number_of_fake_cars) + " " + str(number_of_fake_obstacles) + ".png")  #場所と名前を指定して図を保存
    plt.clf()  #図のクリア

    plt.hist(goal_time_list, bins=50, rwidth=0.9, color='b')  #移動時間のヒストグラムをかく
    plt.xlabel("goal time")  #ラベルの指定  
    plt.ylabel("number of cars")
    plt.savefig(folder_name3 + '/' +"ゴールタイム(" + str(a) + ") " + infilename + " " + str(oppcomm_rate) + " " + str(number_of_cars) + " " + str(number_of_obstacles) + " " + str(number_of_fake_cars) + " " + str(number_of_fake_obstacles) + ".png")  #図の保存
    plt.clf()  #図のクリア

    """plt.hist(number_of_opportunistic_communication_list, bins=50,rwidth=0.9, color='b')
    # plt.show()
    plt.savefig("すれ違い数.png")
    plt.clf()

    plt.hist(number_of_shortest_path_changes_list, bins=50, rwidth=0.9,color='b')
    # plt.show()
    plt.savefig("経路変更数.png")
    plt.clf()"""

    with open(folder_name + '/' + file_name, 'w', newline='') as f:  #読み込み・書き込みいずれの場合も組み込み関数open()でファイルを開く。  'w'で書き込み用でファイルオープン　　newline='' が指定されない場合、クォートされたフィールド内の改行は適切に解釈されず、書き込み時に \r\n を行末に用いる処理系では余分な \r が追加されてしまいます。csv モジュールは独自 (universal) の改行処理を行うため、newline='' を指定することは常に安全です。
      writer = csv.writer(f)   #ユーザが与えたデータをデリミタで区切られた文字列に変換し、与えられたファイルオブジェクトに書き込むための writer オブジェクトを返します　(f)は前行で指定している  前の行で指定したものにcsvファイルの書き込み
      for i in range(number_of_cars):  #車両数の範囲だけiを繰り返す
        writer.writerow([goal_time_list[i], moving_distance_list[i]])  #writerowメソッドの引数はリストです。リストの内容をカンマ区切りでcsvファイルに書き込みます。  ()内のリストの指定したものを,で区切って順に書き込む
    sys.exit(0) # end of simulation, exit.  実行しているプログラムの中でメインプロセスを終了させるための関数


  line1.set_data(xdata, ydata)   #188行目に作った空のline1にデータをセット
  line2.set_data(obstacle_x, obstacle_y)
  line3.set_data(Fxdata, Fydata)
  line4.set_data(fakeobs_x, fakeobs_y)
  title.set_text("Simulation step: " + str(time) + ";  # of cars: " + str(len(cars_list) - number_of_obstacles - number_of_fake_obstacles) + "; goal; " + str(goal_count))   #()ないで指定した文字列でテキストをセット

  return line1, line2, line3, line4, title   #ここまでがanimate(time)の定義

##### main #####
if __name__ == "__main__":   #もし__name__ == "__main__"ならば
  #print(opportunistic_communication_frag)
  # root: xml tree of input file
  root = read_parse_netxml(infilename) #58行目よりxmlファイルの読み込み？
  # x_y_dic: node's x,y pos --> node id
  # DG: Directed graph of road network
  # edge_lanes_list: list of lane instances
  x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list = create_road_network(root)  #68行目より道路ネットワークの作成？
  #print(lane_dic)
  # road_segments_list: list of road segment instances
  road_segments_list = create_road_segments(edge_lanes_list)  #120行目より交差点の作成？？  辺のlistを用いて何かを作ってる

  # create cars
  edges_all_list = DG.edges()   #グラフのすべての辺（道路）のリスト   ()は関数を実行で使う括弧です。
  edges_cars_dic = {}    #車が持っている道路の辞書？
  edges_obstacles_dic = {}  #道路の通行不能個所辞書

  for item in edges_all_list:  #edge_all_listを順に取り出しitemとして１つずつ実行
    edges_obstacles_dic[ item ] = []  #変数名[ ]はリストやタプルの要素を取得するときに使います。   edges_obstacles_dicのitemという要素を初期化 or []内に順に格納   おそらく初期化
    edges_cars_dic[ item ] = []   
  #print(edges_cars_dic)


  obstacles_list = []    #各リストや辞書の初期化   []リスト,{}辞書
  fakeobs_list = []
  obstacle_node_id_list = []
  fakeobs_node_id_list = []
  pair_node_id_list = []
  fakepair_node_id_list = [] #追加
  cars_list = []
  fakecars_list = []
  obstacle_dic = {}

  goal_time_list = [] # 移動完了時間リスト
  number_of_shortest_path_changes_list = [] # 経路変更数リスト
  number_of_opportunistic_communication_list = [] # すれ違い通信数リスト
  moving_distance_list = []#総移動距離リスト
  time_list = []

  avoid_count = 0  #回避カウント　　通行不能個所を回避した回数
  math_count = 0  #演算回数
  passing_comunication = 0  #通信の受け渡し回数
  goal_count = 0   #ゴールカウント

  #edges_all_list = DG.edges()
  #create obstacles
  while True:   #Trueである間繰り返す (無限に繰り返す)
    for i in range(number_of_obstacles):  #number_of_obstaclesの長さだけ繰り返す
      obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node()  #find_obstacle_lane_and_nodeは障害物を見つける関数として定義している
      obstacle = Obstacle(obstacle_node_id, obstacle_lane_id, False)  #別のソースファイルのObstacleから引っ張ってきている？  ここのFalseはfakeflagのこと
      obstacle.init(DG)  #obstacleでinit(要素？)を実行   https://atmarkit.itmedia.co.jp/ait/articles/2306/20/news022.htmlより        
      obstacles_list.append(obstacle)  #リストに要素の追加
      cars_list.append(obstacle)  #cars_listに要素の追加
      edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)  #辞書の[]内で指定した要素にobstacleから追加？？
      edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)   #辞書の[]内で指定した要素にobstacleから追加？？
      obstacle_dic[edge_lanes_list[obstacle_lane_id].node_id_list[1]] = False  #辞書の指定した要素を書き換える？
      #print(obstacle_dic)
    if nx.is_weakly_connected(DG) == True:  #nx.is_weakly_connectedは弱い接続の有向グラフをテストします。有向グラフは、グラフが、節点間のエッジの方向が無視される場合に接続されます。グラフが強く接続されている場合(つまり、グラフが接続されている場合) 方向性を考慮しても)、それは定義上弱いです 接続もされています。
      break

      """
    else:  #作業中
      for i in range(number_of_obstacles):  #number_of_obstaclesの長さだけ繰り返す
        obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node()  #find_obstacle_lane_and_nodeは障害物を見つける関数として定義している
        obstacle = Obstacle(obstacle_node_id, obstacle_lane_id, False)  #別のソースファイルのObstacleから引っ張ってきている？  ここのFalseはfakeflagのこと
        obstacle.init(DG)  #obstacleでinit(要素？)を実行   https://atmarkit.itmedia.co.jp/ait/articles/2306/20/news022.htmlより
        obstacles_list.append(obstacle)  #リストに要素の追加
        cars_list.append(obstacle)  #cars_listに要素の追加
        edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)  #辞書の[]内で指定した要素にobstacleから追加？？
        edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)   #辞書の[]内で指定した要素にobstacleから追加？？
        obstacle_dic[edge_lanes_list[obstacle_lane_id].node_id_list[1]] = False
        for j in range(number_of_fake_obstacles):
          fakeobs_lane_id, fakeobs_node_id = find_fakeobs_lane_and_node()  
          obstacle = Obstacle(fakeobs_node_id, fakeobs_lane_id, True)  
          obstacle.init(DG)  
          obstacles_list.append(obstacle)  
          cars_list.append(obstacle) 
          edges_obstacles_dic[(edge_lanes_list[fakeobs_lane_id].node_id_list[0], edge_lanes_list[fakeobs_lane_id].node_id_list[1])].append(obstacle)
          edges_cars_dic[(edge_lanes_list[fakeobs_lane_id].node_id_list[0], edge_lanes_list[fakeobs_lane_id].node_id_list[1])].append(obstacle)
          obstacle_dic[edge_lanes_list[fakeobs_lane_id].node_id_list[1]] = True
          #fakeobs_node_id_list.append(obstacle_node_id_list[number_of_obstacles + i])
        #print(obstacle_dic)
        #print(fakeobs_node_id_list)
        if nx.is_weakly_connected(DG) == True:   #nx.is_weakly_connectedは弱い接続の有向グラフをテストします
          break
        """
      #number_of_all_obstacles = int(number_of_obstacles) + int(number_of_fake_obstacles)
      #for i in range(number_of_all_obstacles):
        #obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node() 
        #obstacle = Obstacle(obstacle_node_id, obstacle_lane_id, True) 
        #obstacle.init(DG)  
        #fakeobs_list.append(obstacle)
        #cars_list.append(obstacle)
        #edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
        #edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
        
      

  #偽の通行不能箇所(仮)
  while True:
    for i in range(number_of_fake_obstacles):
      obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node()  #右辺はユーザー定義関数の実行？？
      obstacle = Obstacle(obstacle_node_id, obstacle_lane_id, True)  #別のソースフォルダからインポートしたものを実行？
      obstacle.init(DG)  #ユーザーが定義した関数initの実行
      fakeobs_list.append(obstacle)  #偽の通行不能個所のリストに要素を追加
      cars_list.append(obstacle)  #車一覧に要素の追加？　　　　　　　　　　　　　ここ？？？
      edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)  #辞書の[]内で指定した要素にobstacleから追加？？
      edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
      obstacle_dic[edge_lanes_list[obstacle_lane_id].node_id_list[1]] = True  #指定した辞書の要素をTrueに　　452行目とは逆の処理　　なぜ？
      fakeobs_node_id_list.append(obstacle_node_id_list[number_of_obstacles + i])  #偽の通行不能個所のノードidリストに（）内のものを追加

    #print(obstacle_dic)
    #print(fakeobs_node_id_list)
    if nx.is_weakly_connected(DG) == True:   #nx.is_weakly_connectedは弱い接続の有向グラフをテストします  おそらくシミュレーションの道路が連結（どこか袋小路になっていないか確認）
     print(obstacle_dic)
     print(fakeobs_node_id_list)
    if nx.is_weakly_connected(DG) == True:   #nx.is_weakly_connectedは弱い接続の有向グラフをテストします
      break

  

  #車両作成
  DG_copied2 = copy.deepcopy(DG)  #DG_copied2にコピーする　　深いコピーでは、一方のいかなる変更も他方には全く影響を与えない完全に別物のオブジェクトを作ることができます。　　　https://www.headboost.jp/python-copy-deepcopy/#index_id3
  for i in range(len(obstacle_node_id_list)):
    DG_copied2.remove_edge(pair_node_id_list[i],obstacle_node_id_list[i])  #remove_edge(u, v)  Uとvの間のエッジを削除します  経路の計算に含まれないように
    #DG_copied2.remove_node(obstacle_node_id_list[i])
  for i in range(number_of_cars):
    # Reference: https://networkx.github.io/documentation/latest/reference/algorithms/generated/networkx.algorithms.shortest_paths.weighted.dijkstra_path.html
    origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()  #find_OD_node_and_laneは出発地点と目的地をランダムに選ぶ　　135行目より
    while True:
      try:
        shortest_path = nx.dijkstra_path(DG_copied2, origin_node_id, destination_node_id)  #dijkstra_path(G, ソース, ターゲット, 重み='重み')  [ソース]   ソースからターゲットまでの最短加重パスを G 単位で返します。ダイクストラの方法を使用して最短加重経路を計算する グラフ内の 2 つのノード間。
        break
      except Exception:  #例外処理
        origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()   #find_OD_node_and_laneは出発地点と目的地をランダムに選ぶ　　135行目より

    shortest_path = nx.dijkstra_path(DG, origin_node_id, destination_node_id)  #dijkstra_path(G, ソース, ターゲット, 重み='重み')  [ソース]   ソースからターゲットまでの最短加重パスを G 単位で返します。ダイクストラの方法を使用して最短加重経路を計算する グラフ内の 2 つのノード間。
    car = Car(origin_node_id, destination_node_id, destination_lane_id, shortest_path, origin_lane_id, DG, False)  #外部ソースファイルからインポートしたクラスCarを使う
    car.init(DG)  # initialization of car settings
    cars_list.append(car)
    edges_cars_dic[(edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1])].append(car)   #辞書の[]内で指定した要素にcarから追加？？
    if oppcomm_rate * number_of_cars < i: #車両の割合ですれ違いのフラグのon/off
      car.opportunistic_communication_frag = False  #すれ違い通信のoff？？

  #悪意のある車両作成(仮)
  for j in range(number_of_fake_cars):  #悪意を持った車両の数だけ繰り返す
    origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()  #find_OD_node_and_laneは出発地点と目的地をランダムに選ぶユーザー定義関数　　135行目より
    while True:
      try:
        shortest_path = nx.dijkstra_path(DG_copied2, origin_node_id, destination_node_id)   #車両作成の流れと全く同じ  483行目
        break
      except Exception:
        origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()

    shortest_path = nx.dijkstra_path(DG, origin_node_id, destination_node_id)
    car = Car(origin_node_id, destination_node_id, destination_lane_id, shortest_path, origin_lane_id, DG, True)
    car.init(DG)  # initialization of car settings
    cars_list.append(car)
    fakecars_list.append(car)
    edges_cars_dic[(edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1])].append(car)  #ここまで車両作成と同じ
    for j in range(number_of_fake_obstacles):  #偽の通行不能個所ン数だけ繰り返す
      for k, v in obstacle_dic.items():   #各要素のキーkeyと値valueの両方に対してforループ処理を行うには、items()メソッドを使う。 https://note.nkmk.me/python-dict-keys-values-items/
        if v == True and k not in car.obstacles_info_list:  #もし、辞書に対してのvalue(key)じゃない方がTrueでなおかつ、kがcar.obstacles_info_listに入っていない場合
          car.obstacle_dic[k] = True  #辞書のkというkeyをTrueとする  おそらく通行可能という情報を通行不可能に書き変え処理を行っている
          car.obstacles_info_list.append(k)  #車が持っている通行不能個所リストにkを追加
    #print("偽の通行不能箇所の辞書" + str(car.obstacle_dic))
    print(str(car) + "偽の通行不能箇所のリスト" + str(car.obstacles_info_list))  #偽の通行不能個所に関する情報の出力
    if oppcomm_rate * number_of_fake_cars < j: #車両の割合ですれ違いのフラグのon/off
      car.opportunistic_communication_frag = False



  # animation initial settings
  fig, ax = plt.subplots()  #Figure：描画領域全体  Axes：一つ一つのプロットを描く領域  引数を省力：１つのサブプロットを生成  https://www.yutaka-note.com/entry/matplotlib_subplots#:~:text=nrows%3D2%20ncols%3D1%20fig%2C%20axes%20%3D%20plt.subplots%28nrows%3Dnrows%2C%20ncols%3Dncols%2C%20squeeze%3DFalse%2C,in%20range%28ncols%29%3A%20axes%5Bi%2Cj%5D.plot%28x%2C%20y%29%20axes%5Bi%2Cj%5D.set_title%28f%22plot%20%28%7Bi%7D%2C%20%7Bj%7D%29%22%29%20plt.show%28%29
  xdata = []; ydata = []  #リストの初期化
  for i in range(len(cars_list)):  
    xdata.append( cars_list[i].current_position[0] )  #リストに[]で指定した要素を追加　　各車の位置
    ydata.append( cars_list[i].current_position[1] )
  obstacle_x = []; obstacle_y = []  #リストの初期化
  for i in range(len(obstacles_list)):  
    obstacle_x.append(obstacles_list[i].current_position[0])  #リストに[]で指定した要素を追加　　通行不能個所の位置
    obstacle_y.append(obstacles_list[i].current_position[1])
  Fxdata = []; Fydata = []  #リストの初期化
  for i in range(len(fakecars_list)):
    Fxdata.append( fakecars_list[i].current_position[0] )  #リストに[]で指定した要素を追加  悪意もち車両のx軸y軸の位置？
    Fydata.append( fakecars_list[i].current_position[1] )
  fakeobs_x = []; fakeobs_y = []  #リストの初期化
  for i in range(len(fakeobs_list)):
    fakeobs_x.append(fakeobs_list[i].current_position[0])  #リストに[]で指定した要素を追加  偽の通行不能個所の位置
    fakeobs_y.append(fakeobs_list[i].current_position[1])


  line1, = plt.plot([], [], color="green", marker="s", linestyle="", markersize=5)  #グラフの描画　色は緑　　https://qiita.com/ground0state/items/f415d771fd1062f81830
  line2, = plt.plot([], [], color="red", marker="s", linestyle="", markersize=5)
  line3, = plt.plot([], [], color="blue", marker="s", linestyle="", markersize=5)
  line4, = plt.plot([], [], color="cyan", marker="s", linestyle="", markersize=5)
  title = ax.text(20.0, -20.0, "", va="center")  #https://python.atelierkobato.com/text/  最初の数字二つは文字を表示する座標？　　文字を表示する

  #img = Image.open(png_infilename)
  #img_list = np.asarray(img)
  #plt.imshow(img_list)
  ## draw road network
  draw_road_network(DG)  #181行目より、draw_road_networkという関数を使ってネットワークを描画


  #print("通行不能箇所の辞書" + str(obstacle_dic))
  #print("remath:" + str(math_count) + " through:" + str(avoid_count) + " pass:" + str(passing_comunication))
  print("### Start of simulation ###")
  ani = FuncAnimation(fig, animate, frames=range(1000), init_func=init, blit=True, interval= 10)  #アニメーションを作成するのに必要な情報を渡せばアニメーションを作成してくれます
  #ani.save("grid-sanimation.gif", writer='imagemagick')
  plt.show()  #複数のFigureを表示したいときに、使えたりします  実行されたとき始めてウィンドウが立ち上がり、そのウィンドウにグラフが表示されるといった状況になるはず

