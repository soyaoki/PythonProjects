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

    # グリッドマップを取得し、描画する関数
    def draw_grid_map(self):
        url_grid = 'http://192.168.1.23:10009/get/cleaning_grid_map'
        grid_data = self.send_request(url_grid)
        lower_left_x = grid_data.get('lower_left_x')
        lower_left_y = grid_data.get('lower_left_y')
        size_x = grid_data.get('size_x')
        size_y = grid_data.get('size_y')
        resolution = grid_data.get('resolution')
        cleaned = grid_data.get('cleaned')
        
        # plt.figure(figsize=(size_x * resolution / 100, size_y * resolution / 100))

        if (len(cleaned)>0):
            map_data = [['cleaned' for _ in range(size_x)] for _ in range(size_y)]
            x, y = 0, 0

            for i in range(len(cleaned)):
                if i % 2 == 0:
                    x += cleaned[i]
                else:
                    for _ in range(cleaned[i]):
                        map_data[y][x] = 'not cleaned'
                        x += 1
                        if x == size_x:
                            x = 0
                            y += 1
                            if y == size_y:
                                break

            for i in range(size_y):
                for j in range(size_x):
                    if map_data[i][j] == 'cleaned':
                        plt.fill([lower_left_x + j * resolution, lower_left_x + (j+1) * resolution, lower_left_x + (j+1) * resolution, lower_left_x + j * resolution], 
                                    [lower_left_y + i * resolution, lower_left_y + i * resolution, lower_left_y + (i+1) * resolution, lower_left_y + (i+1) * resolution], 
                                    color='blue', alpha=0.5)

        plt.xticks(np.arange(lower_left_x, lower_left_x + (size_x + 1) * resolution, resolution))
        plt.yticks(np.arange(lower_left_y, lower_left_y + (size_y + 1) * resolution, resolution))
        
    
    # 特徴マップを取得し、地図情報を描画する関数
    def draw_map(self):
        url_map = 'http://192.168.1.23:10009/get/feature_map'
        map_data = self.send_request(url_map)
        map_lines = map_data.get('map').get('lines')
        docking_pose = map_data.get('map').get('docking_pose')
        for line in map_lines:
            x1 = line.get('x1')
            y1 = line.get('y1')
            x2 = line.get('x2')
            y2 = line.get('y2')
            plt.plot([x1, x2], [y1, y2],color='gray')
        plt.plot([docking_pose.get('x')], [docking_pose.get('y')], marker="D", markersize=6) #ドック

        url_polygons = 'http://192.168.1.23:10009/get/n_n_polygons'
        polygons_data = self.send_request(url_polygons)
        polygons = polygons_data.get('map').get('polygons')

        # for文
        l=len(polygons)
        for i in range(0,l):
            x = []
            y = []
            seg = polygons[i].get('segments')
            m = len(seg)
            for ii in range(0,m):
                x1 = seg[ii].get('x1')
                y1 = seg[ii].get('y1')
                x2 = seg[ii].get('x2')
                y2 = seg[ii].get('y2')
                x = [x1,x2]
                y = [y1,y2]
                plt.plot(x, y, color='gray')

    
    # ロボットの位置を取得し、現在位置を描画する関数
    def draw_robot_position(self):
        url_pose = 'http://192.168.1.23:10009/get/rob_pose'
        pose_data = self.send_request(url_pose)
        x1 = pose_data.get('x1')
        y1 = pose_data.get('y1')
        x2 = pose_data.get('x2')
        y2 = pose_data.get('y2')
        plt.plot([x1, x2], [y1, y2], marker="*",markersize=12) #イマココ
    
    # ノードのメイン処理
    def main_loop(self):
        plt.clf()  # グラフをクリアして新しいフレームを描画する準備をする
        self.draw_grid_map()
        self.draw_map()
        self.draw_robot_position()
        plt.grid()
        plt.axis('equal')
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
