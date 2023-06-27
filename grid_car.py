import networkx as nx
import numpy as np
import math
import copy
import pprint #リストなどを整列して出力や文字列に変換をするライブラリ

class Car:
  def __init__(self, orig_node_id, dest_node_id, dest_lane_id, shortest_path, current_lane_id, DG, fakecar_flag): #__init__はクラスの初期化メソッド、selfは必須の第一引数。クラスのインスタンス自身を指す。以降は引数。
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
    self.number_of_opportunistic_communication = 0    #すれ違い通信数
    self.elapsed_time = 0 #経過時間
    self.moving_distance = 0  #移動距離
    self.goal_arrived = False   #ゴール到着はFalse
    self.DG_copied = copy.deepcopy(DG)   #ディープコピーも一方のオブジェクトをもう一方に複製する動作　　オブジェクトがリストなどの属性を持っていたとして、シャシャローコピーはそのリストのコピーには参照を使うが、ディープコピーはリストの要素を再帰的にコピーして複製　DGをコピーする
    self.opportunistic_communication_frag = True  #すれ違い通信をするかどうか

    self.short_path = []  #最短経路?
    self.obstacle_dic = {}   #通行不能個所の空の辞書？
    self.fakecar_flag = fakecar_flag   #fakecar_flag  おそらく悪意を持った車両

  #車線に関するパラメータ
  def init(self, DG):  #grid_simulatarの後半でよく使われてたユーザー定義のinit関数
    current_start_node_id = self.shortest_path[ self.current_sp_index ]  #current_start_node_id(現在の開始位置)に最短経路のindex  「index」とは、list型変数のある要素がlistの中で何番目かを示すもので、もし指定された値が存在しなければエラーを起こします。https://www.tech-teacher.jp/blog/python-index/
    self.current_start_node = DG.nodes[ current_start_node_id ]["pos"]  #ノード属性を追加する  ノードを追加してもグラフには追加されず、新しいノードを追加するために使用されます  https://networkx.org/documentation/stable/tutorial.html
    self.current_position = DG.nodes[ current_start_node_id ]["pos"]
    current_end_node_id = self.shortest_path[ self.current_sp_index+1]
    self.current_end_node = DG.nodes[ current_end_node_id ]["pos"]
    current_edge_attributes = DG.get_edge_data(current_start_node_id, current_end_node_id)  #current_edge_attributes = 現在のedgeの属性　　　　エッジ (u, v) に関連付けられた属性ディクショナリ（辞書？）を返す
    self.current_max_speed = current_edge_attributes["speed"]  #current_edge_attributesの[""]で指定したものへアクセス？
    self.current_distance = current_edge_attributes["weight"]

  # Optimal Velocity Function to determine the current speed
  #最適速度関数
  def V(self, inter_car_distance):
    return 0.5*self.current_max_speed*(np.tanh(inter_car_distance-2) + np.tanh(2)) #車間距離から自身の速度を決定する

  # update car's speed
  #inter_car_distance = diff_dist
  #車両の速度の更新
  def update_current_speed(self, sensitivity, inter_car_distance):
    self.current_speed += sensitivity*( self.V(inter_car_distance) - self.current_speed )

  def move(self, edges_cars_dic, sensitivity, lane_dic, edge_length_dic):

    self.elapsed_time += 1  #elapsed_time = 経過時間
    #self.moving_distance += edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]]
    #print(edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]])
    # x_prev == self.current_position[0]
    # y_prev == self.current_position[1]
    direction_x = self.current_end_node[0] - self.current_position[0]  #x軸方向を現在の終了ノード　－　現在地　で求める？
    direction_y = self.current_end_node[1] - self.current_position[1]  #y軸方向
    arg = math.atan2(direction_y, direction_x)  #正接、逆正接 arctanを求めてる　　引数が二つで、"arctan(y / x)"をラジアンで返す。この角度は、極座標平面において原点から座標(x, y)へのベクトルがx軸の正の方向となす角度（偏角）であり、戻り値は-piからpi（-180度から180度）の間になる。

    arrived_cars_list = []  #到着した車のリストの初期化

    #x_new = None; y_new = None
    #print(edge_length_dic.get(lane_dic[self.shortest_path[self.current_sp_index]]))
    #pprint.pprint(lane_dic)
    #print(self.shortest_path)

    #print(self.moving_distance)
    #edge_length_dic  key : lane_id, value : edge_length
    if np.sqrt((self.current_position[0] - self.current_end_node[0])**2 + (self.current_position[1] - self.current_end_node[1])**2) < self.current_speed: #車線の終点（最初からコメントしてあった）　　平方根　　https://www.delftstack.com/ja/api/numpy/python-numpy-sqrt/　　a**2はaを２回かける
      if self.shortest_path[self.current_sp_index] in lane_dic:  #lane_dicがself.shortest_path[self.current_sp_index]に含まれているとき
        self.moving_distance += edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]] #前のノードから移動した距離
      self.current_sp_index += 1
      if self.current_sp_index >= len(self.shortest_path)-1: #目的地に着いたときの処理をする
        self.goal_arrived = True
        #現在地を更新
        x_new = self.current_end_node[0]
        y_new = self.current_end_node[1]
        #今いる車線の始点,終点,番号を求める
        current_start_node_id = self.shortest_path[ self.current_sp_index-1 ]
        current_end_node_id = self.shortest_path[ self.current_sp_index ]
        self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]
        #前の車両を求める
        car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
        car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
        diff_dist = 50.0

        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].remove( self )  #始点、終点の削除
        arrived_cars_list.append( self )

      else: #車線変更
        if self.shortest_path[self.current_sp_index] in lane_dic:
          self.moving_distance += edge_length_dic[lane_dic[self.shortest_path[self.current_sp_index]]]
        x_new = self.current_end_node[0]
        y_new = self.current_end_node[1]

        current_start_node_id = self.shortest_path[ self.current_sp_index-1 ]  #85行目と同じ
        current_end_node_id = self.shortest_path[ self.current_sp_index ]  #86行目と同じ
        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].remove( self )  #93行目と同じ　なぜ？
        #変更後車線の始点,終点,現在地を求め,車両を追加する
        current_start_node_id = self.shortest_path[ self.current_sp_index ]  #35～42行目と同じ処理　ただし、グラフはコピーしたもの？？
        self.current_start_node = self.DG_copied.nodes[ current_start_node_id ]["pos"]  #ノード属性を追加する  グラフのコピーしたものに？？
        self.current_position = self.DG_copied.nodes[ current_start_node_id ]["pos"]
        current_end_node_id = self.shortest_path[ self.current_sp_index+1]
        self.current_end_node = self.DG_copied.nodes[ current_end_node_id ]["pos"]
        current_edge_attributes = self.DG_copied.get_edge_data(current_start_node_id, current_end_node_id)  #ここまでおなじ
        #self.current_max_speed = current_edge_attributes["speed"]
        #self.current_distance = current_edge_attributes["weight"]  #ここまでおなじ（コメントアウト済み）
        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].append( self )  #引数のselfって何？？？
        #print(edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)) #レーン内の車両または障害物の数

        if edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) > 0:  
          car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) - 1  #前方車両について
          car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
          diff_dist = 50

        else:
          car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
          car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
          diff_dist = 50.0  #diff→二つの違いを比較するなど？　　dist→おそらく距離

    else: #現在の車線の終点まで　　７５行目の例外処理
      x_new = self.current_position[0] + self.current_speed*np.cos(arg)  
      y_new = self.current_position[1] + self.current_speed*np.sin(arg)
      self.current_position = [x_new, y_new]
      current_start_node_id = self.shortest_path[ self.current_sp_index ]
      current_end_node_id = self.shortest_path[ self.current_sp_index+1 ]

      if edges_cars_dic[ (current_start_node_id, current_end_node_id) ].index( self ) > 0:
        car_forward_index = edges_cars_dic[ (current_start_node_id, current_end_node_id) ].index( self ) - 1  #前方車両について
        car_forward_pt = edges_cars_dic[ (current_start_node_id, current_end_node_id) ][ car_forward_index ]
        diff_dist = np.sqrt( (car_forward_pt.current_position[0] - self.current_position[0])**2 + (car_forward_pt.current_position[1] - self.current_position[1])**2 )

      else:
        car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)  #前方車両について
        car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
        diff_dist = 50.0
      self.update_current_speed(sensitivity, diff_dist)   #車両の速度の更新（ユーザー定義）を使う？　５２行目より　引数は()のもの
      if self.shortest_path[self.current_sp_index] not in lane_dic:  #lane_dicにself.shortest_path[self.current_sp_index]がない場合　　おそらく最短経路でだした道がない場合
        None
        #print("KeyError : " + str(self.shortest_path[self.current_sp_index]) + "sub" + str(self.current_sp_index) + str(self))
        #print(self.shortest_path)
      else:
        self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]  #現在の道を更新
    return x_new, y_new, self.goal_arrived, car_forward_pt, diff_dist

  def U_turn(self, edges_cars_dic,lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list):  #Uターンのための関数を定義
    self.current_sp_index += 1 

    x_new = self.current_end_node[0]
    y_new = self.current_end_node[1]

    # Uターン前のモデルの削除　　更新？
    current_start_node_id = self.shortest_path[self.current_sp_index - 1]
    current_end_node_id = self.shortest_path[self.current_sp_index]
    edges_cars_dic[(current_start_node_id, current_end_node_id)].remove(self)
    pre_start_node_id = current_start_node_id
    pre_end_node_id = current_end_node_id

    #発見した障害物ノードidを保存
    if current_end_node_id not in self.obstacles_info_list:
      self.obstacles_info_list.append(current_end_node_id)
      #print(self.obstacles_info_list)
    #現在の車線番号
    self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]
    #print("今のノードの番号:"+str(self.shortest_path[self.current_sp_index]))
    #print("今いるレーンの番号:"+str(self.current_lane_id))

    #対向車線を求め, 始点,終点,現在地を求める
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

    self.current_start_node = self.DG_copied.nodes[current_start_node_id]["pos"]  #ノードの属性付け
    self.current_position = self.DG_copied.nodes[current_start_node_id]["pos"]
    self.current_end_node = self.DG_copied.nodes[current_end_node_id]["pos"]
    #print(x_y_dic[self.current_position])#ノード番号

    #障害物を含むedgeの削除
    if self.DG_copied.has_edge(pre_start_node_id, pre_end_node_id) == True:  #エッジ (u, v) がグラフ内にある場合は True を返します
      self.DG_copied.remove_edge(pre_start_node_id, pre_end_node_id)

    #最短経路の再計算
    #print("車線変更前のshortest_path"+str(self.shortest_path))
    while True:
      try:
        self.shortest_path = nx.dijkstra_path(self.DG_copied, current_start_node_id, self.dest_node_id)  #最短経路の計算
        break
      except Exception:  #例外処理
          self.dest_lane_id = np.random.randint(len(edge_lanes_list))  #dest = 目的地　ランダムに決める
          self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]  #指定した辞書の要素を目的地のノードに
          while self.dest_node_id in obstacle_node_id_list or self.current_lane_id == self.dest_lane_id: #目的地ノードが通行不能個所リストにある場合、または現在のレーンと目的地レーンが同じ場合
            self.dest_lane_id = np.random.randint(len(edge_lanes_list))  #目的地の決めなおし
            self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]  #目的地までのレーンの決めなおし

    #print("車線変更後のshortest_path" + str(self.shortest_path))
    self.number_of_shortest_path_changes += 1  #最短経路の変更回数
    self.current_sp_index = 0# current_sp_indexのリセット

    current_start_node_id = self.shortest_path[self.current_sp_index]  #ここから下はなぜか35行目から42行目と同じことをしてる　グラフから用いるものはコピーしたグラフを使ってる
    self.current_start_node = self.DG_copied.nodes[current_start_node_id]["pos"]
    self.current_position = self.DG_copied.nodes[current_start_node_id]["pos"]
    current_end_node_id = self.shortest_path[self.current_sp_index + 1]
    self.current_end_node = self.DG_copied.nodes[current_end_node_id]["pos"]
    current_edge_attributes = self.DG_copied.get_edge_data(current_start_node_id, current_end_node_id)
    self.current_max_speed = current_edge_attributes["speed"]
    self.current_distance = current_edge_attributes["weight"]
    edges_cars_dic[(current_start_node_id, current_end_node_id)].append(self)  #辺の車の辞書に追加  edge　＝　端っこという意味？？
    #print('U_turn end!')

    return x_new, y_new
