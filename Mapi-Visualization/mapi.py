import os.path
import numpy as np
import rclpy
from rclpy.node import Node
import requests
import matplotlib.pyplot as plt

class CleaningRobotNode(Node):
    def __init__(self):
        super().__init__('cleaning_robot_node')
        
    # HTTPリクエストを送信する関数
    def send_request(self, url):
        req = requests.get(url)
        return req.json()
    
    # 特徴マップを取得し、地図情報を描画する関数
    def draw_map(self):
        url_map = 'http://192.168.1.23:10009/get/feature_map'
        map_data = self.send_request(url_map)
        map_lines = map_data.get('map').get('lines')
        for line in map_lines:
            x1 = line.get('x1')
            y1 = line.get('y1')
            x2 = line.get('x2')
            y2 = line.get('y2')
            plt.plot([x1, x2], [y1, y2])
    
    # ロボットの位置を取得し、現在位置を描画する関数
    def draw_robot_position(self):
        url_pose = 'http://192.168.1.23:10009/get/rob_pose'
        pose_data = self.send_request(url_pose)
        x1 = pose_data.get('x1')
        y1 = pose_data.get('y1')
        x2 = pose_data.get('x2')
        y2 = pose_data.get('y2')
        plt.plot([x1, x2], [y1, y2], marker="*")
    
    # グリッドマップを取得し、描画する関数
    def draw_grid_map(self):
        url_grid = 'http://192.168.1.23:10009/get/cleaning_grid_map'
        grid_data = self.send_request(url_grid)
        xmin = grid_data.get('lower_left_x')
        ymin = grid_data.get('lower_left_y')
        xsize = grid_data.get('size_x')
        ysize = grid_data.get('size_y')
        res = grid_data.get('resolution')
        plt.xticks(list(filter(lambda x: x%40==0, np.arange(xmin,xmin + xsize * res))))
        plt.yticks(list(filter(lambda y: y%40==0, np.arange(ymin,ymin + ysize * res))))
        plt.grid()
    
    # ノードのメイン処理
    def main_loop(self):
        plt.clf()  # グラフをクリアして新しいフレームを描画する準備をする
        self.draw_map()
        self.draw_robot_position()
        self.draw_grid_map()
        plt.savefig('result.png')
        plt.pause(1)  # 1秒待機する

def main(args=None):
    rclpy.init(args=args)
    node = CleaningRobotNode()
    timer = node.create_timer(1.0, node.main_loop)  # 1秒ごとにmain_loop()を呼び出すタイマーを作成
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
