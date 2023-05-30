import grid_car  #grid_carファイルの取り込み
class RoadSegment:  #クラス作成   新しいデータの型を作る
  def __init__(self, lane1, lane2):  #defで新しく定義  _init_は「インスタンス化」する際に必要なメソッド
    self.lane1 = lane1   #引数として受け取った値「lane1」をインスタンス変数 「lane1」へ代入する
    self.lane2 = lane2   #引数として受け取った値「lane2」をインスタンス変数 「lane2」へ代入する