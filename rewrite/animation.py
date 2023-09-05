#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
from functions import *
from Car import *
from Obstacle import *

fig = plt.subplot()
ax = plt.subplot()

line1, = plt.plot([], [], color="green", marker="s", linestyle="", markersize=5)
line2, = plt.plot([], [], color="red", marker="s", linestyle="", markersize=5)
line3, = plt.plot([], [], color="blue", marker="s", linestyle="", markersize=5)
line4, = plt.plot([], [], color="cyan", marker="s", linestyle="", markersize=5)
title = ax.text(20.0, -20.0, "", va="center")

def animate(time):
    global xdata, ydata, Fxdata,Fydata,avoid_count,math_count,passing_comunication,goal_count #xdata, ydata = carのx,yのid?  passing_comunication = すれ違い通信
    global goal_time_list, number_of_shortest_path_changes_list, number_of_opportunistic_communication_list, moving_distance_list, time_list
    
    xdata = [], ydata = []
    Fxdata = [],Fydata = [] #fakecarのx, y id

    for num_car in cars_list:
        if num_car.__class__.__name__ == 'Car':
            time_list.append(time) #シミュレータの経過時間を格納？
            x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = num_car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)
        elif num_car.__class__.__name__ == 'fakeCar':
            time_list.append(time)
            x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = num_car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)

        if num_car.goal_arrived == True:  #車がgoal到着しているなら
            goal_count += 1
            #number_of_shortest_path_changes_list.append(num_car.number_of_shortest_path_changes) #最短経路変更回数
            #number_of_opportunistic_communication_list.append(num_car.number_of_opportunistic_communication) すれ違い通信数  未実装？
            goal_time_list.append(num_car.elapsed_time) #elapsed = 経過時間
            moving_distance_list.append(round(num_car.moving_distance,1)) #移動距離をリストに追加　四捨五入済み

            if type(num_car) == Car:
                cars_list.remove( num_car )

            elif type(num_car) == fake_Car:
                cars_list.remove( num_car )   
                print("悪意のある車の削除")

        
        #障害物があればUターン
        if car_forward_pt.__class__.__name__ != "Car" and car_forward_pt.__class__.__name__ != "fakeCar" and diff_dist <= 20: #前方にあるのが車じゃない（障害物なら）　なおかつ　その距離（車間距離）が20以下なら
            if type(car_forward_pt) == Obstacle:
                x_new, y_new = num_car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list) # 新しいx, y　のidに（Uターンする）

            #偽の障害物なら通過
            elif type(car_forward_pt) == fake_Obstacle:
                None
        
        #TODO 現在は前方のcar_forward_ptがCarクラスのインスタンスではないとき、このif文が処理されてcar_forward_ptのfake_flagで条件分岐し、Uturnするようになっている
        #ただ今後の予定ではfake_flagで一般車両か、悪意を持った車両なのかを管理するのではなく、Carクラスとfake_Carクラスで区別する予定なのでfake_flagは削除予定
        #よって、Uturnするときの条件をcar_forward_ptで管理するのではなくて、Obstacleクラスで管理するように定義したい

def plot_car_and_obstacle(cars_list,edges_cars_dic, sensitivity, lane_dic, edge_length_dic,obstacles_list,fake_obstacles_list,edge_lanes_list, x_y_dic, obstacle_node_id_list):
    global line1, line2, line3, line4
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    title.set_text("Simulation step: 0")

    xdata = []
    ydata = []
    Fxdata = []
    Fydata = []
    obstacle_x = [] 
    obstacle_y = []
    fake_obstacle_x = []
    fake_obstacle_y = []

    for i in cars_list:
        x_new, y_new = i.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)
        if car_forward_pt.__class__.name__ != "Car" and car_forward_pt.__class__.name__ != "fake_Car" and diff_dist <= 20:
            if type(car_forward_pt) == Obstacle:
                x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list)

        xdata.append(x_new)
        ydata.append(y_new)
        if type(cars_list[i]) == Car:
            Fxdata.append(x_new)
            Fydata.append(y_new)

    for j in obstacles_list:
        x_new = j.current_position[0]   
        y_new = j.current_position[1]
        obstacle_x.append(x_new)
        obstacle_y.append(y_new)

    for k in fake_obstacles_list:
        x_new = k.current_position[0]
        y_new = k.current_position[1]
        fake_obstacle_x.append(x_new)
        fake_obstacle_y.append(y_new)

    #データセット
    line1.set_data(xdata, ydata)
    line2.set_data(obstacle_x, obstacle_y)
    line3.set_data(Fxdata, Fydata)
    line4.set_data(fake_obstacle_x, fake_obstacle_y)
    title.set_text("Simulation step: " + str(time) + ";  # of cars: " + str(len(cars_list) - number_of_obstacles - number_of_fake_obstacles) + "; goal; " + str(goal_count))

    """""
  # animation initial settings
  fig, ax = plt.subplots()  #Figure:描画領域全体  Axes:一つ一つのプロットを描く領域  引数を省力：１つのサブプロットを生成  https://www.yutaka-note.com/entry/matplotlib_subplots#:~:text=nrows%3D2%20ncols%3D1%20fig%2C%20axes%20%3D%20plt.subplots%28nrows%3Dnrows%2C%20ncols%3Dncols%2C%20squeeze%3DFalse%2C,in%20range%28ncols%29%3A%20axes%5Bi%2Cj%5D.plot%28x%2C%20y%29%20axes%5Bi%2Cj%5D.set_title%28f%22plot%20%28%7Bi%7D%2C%20%7Bj%7D%29%22%29%20plt.show%28%29
  xdata = []; ydata = []  
  for i in range(len(cars_list)):  
    xdata.append( cars_list[i].current_position[0] )  
    ydata.append( cars_list[i].current_position[1] )
  obstacle_x = []; obstacle_y = []  #リストの初期化
  for i in range(len(obstacles_list)):  
    obstacle_x.append(obstacles_list[i].current_position[0]) 
    obstacle_y.append(obstacles_list[i].current_position[1])
  Fxdata = []; Fydata = []  #リストの初期化
  for i in range(len(fakecars_list)):
    Fxdata.append( fakecars_list[i].current_position[0] )  
    Fydata.append( fakecars_list[i].current_position[1] )
  fakeobs_x = []; fakeobs_y = []  #リストの初期化
  for i in range(len(fakeobs_list)):
    fakeobs_x.append(fakeobs_list[i].current_position[0])  
    fakeobs_y.append(fakeobs_list[i].current_position[1])
    """""


    #アニメーションの描画
    """line1, = plt.plot([], [], color="green", marker="s", linestyle="", markersize=5)
    line2, = plt.plot([], [], color="red", marker="s", linestyle="", markersize=5)
    line3, = plt.plot([], [], color="blue", marker="s", linestyle="", markersize=5)
    line4, = plt.plot([], [], color="cyan", marker="s", linestyle="", markersize=5)
    title = ax.text(20.0, -20.0, "", va="center")"""

    draw_road_network(DG) #道路ネットワークの読み込み

    print("### Start of simulation ###")
    ani = FuncAnimation(fig, animate, frames=range(1000), init_func=init, blit=True, interval= 10)
    plt.show()
