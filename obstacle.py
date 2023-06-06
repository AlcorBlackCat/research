import networkx as nx
import numpy as np
import math

class Obstacle:   #クラスの定義    この中では通行不能個所についてのプログラム　　fake_flagは偽情報アリか無しかのフラグ
  def __init__(self,obstacle_node_id, obstacle_lane_id, fake_flag):   #_init_は「インスタンス化」する際に必要なメソッド このなかではobstacle_node_id, obstacle_lane_id, fake_flagを定義してる？
    self.fake_flag = fake_flag   #selfはインスタンス自身を表す
    self.current_position  = []
    if fake_flag == False:     #もしfake__flagがFalseなら(偽情報がないなら？)
      self.obstacle_node_id = obstacle_node_id   #それぞれ左のインスタンスは右で受け取ったもの
      self.obstacle_lane_id = obstacle_lane_id
    else:       #それ以外
      self.fakeobs_node_id = obstacle_node_id  #それぞれ左のインスタンスは右で受け取ったもの
      self.fakeobs_lane_id = obstacle_lane_id


  def init(self, DG):     #DGの定義
    if self.fake_flag == False:    #もしfake__flagがFalseなら(偽情報がないなら？)
      current_node_id = self.obstacle_node_id   #current_node_idはobstacle_node_idというインスタンス
      #print("通行不能箇所" + str(current_node_id))
    else:                                     #それ以外ならcurrent_node_idはfakeobs_node_id 
      current_node_id = self.fakeobs_node_id    #current_node_idにfakeobs_node_id（通行不能個所）インスタンスを入れる
      #print("偽の通行不能箇所" + str(current_node_id))
    self.current_position = DG.nodes[ current_node_id ]["pos"]   #DG = nx.DiGraph() Directed graph of road network?  DGのnodeの[][]で指定した要素を持ってくる

  def move(self):   #moveの定義
    x_new = self.current_position[0]   #x_newはcurrent_positionの0の属性
    y_new = self.current_position[1]    #y_newはcurrent_positionの1の属性
    return x_new, y_new    #返り値をx_newとy_newに格納   (x_new,y_newを使うとこれらの処理の結果が用いられる)
