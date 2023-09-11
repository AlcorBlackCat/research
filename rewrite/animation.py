#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
from functions import *
from Car import *
from Obstacle import *

fig, ax = plt.subplots()

line1, = plt.plot([], [], color="green", marker="s", linestyle="", markersize=5)
line2, = plt.plot([], [], color="red", marker="s", linestyle="", markersize=5)
line3, = plt.plot([], [], color="blue", marker="s", linestyle="", markersize=5)
line4, = plt.plot([], [], color="cyan", marker="s", linestyle="", markersize=5)
title = ax.text(20.0, -20.0, "", va="center")

def animate(time):
    global xdata, ydata, Fxdata,Fydata,avoid_count,math_count,passing_comunication,goal_count
    global goal_time_list, number_of_shortest_path_changes_list, number_of_opportunistic_communication_list, moving_distance_list, time_list, lane_dic, edge_length_dic, edges_cars_dic
    
    xdata = []; ydata = []
    Fxdata = []; Fydata = [] 

    for num_car in cars_list:
        if num_car.__class__.__name__ == 'Car':
            time_list.append(time) 
            x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = num_car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)
        elif num_car.__class__.__name__ == 'fakeCar':
            time_list.append(time)
            x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = num_car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)

        if num_car.goal_arrived == True:
            goal_count += 1
            goal_time_list.append(num_car.elapsed_time)
            moving_distance_list.append(round(num_car.moving_distance,1))

            if type(num_car) == Car:
                cars_list.remove( num_car )

            elif type(num_car) == fake_Car:
                cars_list.remove( num_car )   
                print("攻撃車両の削除")

        

        if car_forward_pt.__class__.__name__ != "Car" and car_forward_pt.__class__.__name__ != "fakeCar" and diff_dist <= 20: 
            if type(car_forward_pt) == Obstacle:
                x_new, y_new = num_car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list) 
            elif type(car_forward_pt) == fake_Obstacle:
                None

    return xdata, ydata, Fxdata, Fydata
        


def plot_car_and_obstacle(cars_list,edges_cars_dic, sensitivity, lane_dic, edge_length_dic,obstacles_list,fake_obstacles_list,edge_lanes_list, x_y_dic, obstacle_node_id_list):
    global line1, line2, line3, line4, fig, ax
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    title.set_text("Simulation step: 0")

    time = 0
    goal_count = 0

    xdata = []
    ydata = []
    Fxdata = []
    Fydata = []
    obstacle_x = [] 
    obstacle_y = []
    fake_obstacle_x = []
    fake_obstacle_y = []

    for i in cars_list:
        x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = i.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)
        if car_forward_pt.__class__.__name__ != "Car" and car_forward_pt.__class__.name__ != "fake_Car" and diff_dist <= 20:
            if type(car_forward_pt) == Obstacle:
                x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list)

        xdata.append(x_new)
        ydata.append(y_new)
        if type (i) == fake_Car:
            Fxdata.append(x_new)
            Fydata.append(y_new)
        
        time += 1 

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
    title.set_text("Simulation step: " + str(time) + ";  # of cars: " + str(len(cars_list)) + "; goal: " + str(goal_count))


    draw_road_network(DG)

    print("### Start of simulation ###")
    ani = FuncAnimation(fig, animate, frames=range(1000), blit=True, interval= 10)
    plt.show()
