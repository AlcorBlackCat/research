class Lane:
  def __init__(self, from_id, to_id, lane_id):
        self.from_id = from_id
        self.to_id = to_id
        self.lane_id = lane_id  # 追加: レーンのID
        self.node_x_list, self.node_y_list = self.get_lane_node_list(from_id, to_id)
        self.length = self.calculate_length()

  def add_from_to(self, from_id, to_id):
    self.from_id = from_id
    self.to_id = to_id

  def set_others(self, speed, node_id_list, node_x_list, node_y_list):
    self.speed = speed
    self.node_id_list = node_id_list
    self.node_x_list = node_x_list
    self.node_y_list = node_y_list

