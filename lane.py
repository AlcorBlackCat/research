class Lane:    #クラス作成
  def __init__(self):   #defで新しく定義  _init_は「インスタンス化」する際に必要なメソッド
    self.car_list = []     #car.listで受け取った値を格納していく
    self.pheromone = 0     #pheromoreは0

  def add_from_to(self, from_id, to_id):   #sdd_from_toを定義
    self.from_id = from_id   #from_idでうけとったものをfromidに
    self.to_id = to_id    #to_idでうけとったものをto_idに

  def set_others(self, speed, node_id_list, node_x_list, node_y_list):  #set_othersを定義
    self.speed = speed
    self.node_id_list = node_id_list
    self.node_x_list = node_x_list
    self.node_y_list = node_y_list

