def animate(time):
    global xdata, ydata, Fxdata,Fydata,avoid_count,math_count,passing_comunication,goal_count #xdata, ydata = carのx,yのid?  passing_comunication = すれ違い通信
    global goal_time_list, number_of_shortest_path_changes_list, number_of_opportunistic_communication_list, moving_distance_list, time_list
    
    xdata = [], ydata = []
    Fxdata = [],Fydata = [] #fakecarのx, y id

    for num_car in cars_list:
        if num_car.__class__.__name__ == 'Car':
            time_list.append(time) #シミュレータの経過時間を格納？
            x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = num_car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)   

        if num_car.goal_arrived == True:  #車がgoal到着しているなら
            goal_count += 1
            number_of_shortest_path_changes_list.append(num_car.number_of_shortest_path_changes) #最短経路変更回数
            #number_of_opportunistic_communication_list.append(num_car.number_of_opportunistic_communication) すれ違い通信数  未実装？
            goal_time_list.append(num_car.elapsed_time) #elapsed = 経過時間
            moving_distance_list.append(round(num_car.moving_distance,1)) #移動距離をリストに追加　四捨五入済み

            if type(cars_list[num_car]) == Car:
                cars_list.remove( car )

            elif type(cars_list[num_car]) == fake_Car:
                cars_list.remove( car )   
                print("悪意のある車の削除")

        
        #障害物があればUターン
        if car_forward_pt.__class__.__name__ != "Car" and diff_dist <= 20: #前方にあるのが車じゃない（障害物なら）　なおかつ　その距離（車間距離）が20以下なら
            if type(car_forward_pt) == Obstacle:
                x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list) # 新しいx, y　のidに（Uターンする）

            #偽の障害物なら通過
            elif type(car_forward_pt) == fake_Obstacle:
                None
        
        #TODO 現在は前方のcar_forward_ptがCarクラスのインスタンスではないとき、このif文が処理されてcar_forward_ptのfake_flagで条件分岐し、Uturnするようになっている
        #ただ今後の予定ではfake_flagで一般車両か、悪意を持った車両なのかを管理するのではなく、Carクラスとfake_Carクラスで区別する予定なのでfake_flagは削除予定
        #よって、Uturnするときの条件をcar_forward_ptで管理するのではなくて、Obstacleクラスで管理するように定義したい