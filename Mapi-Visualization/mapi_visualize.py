import os.path
import numpy as np
import requests
import matplotlib.pyplot as plt

#ロボット掃除機の情報取得（名前など）
#/get/robot_id

#ステータスの取得（バッテリー残量など）
#/get/status

#イベントログの取得（全部）
#/get/event_log?last_id=0

#掃除開始
#/set/clean_start_or_continue?cleaning_parameter_set=1
#/set/clean_start_or_continue?cleaning_parameter_set=2 気持ち静か

#スポット掃除開始
#/set/clean_spot?map_id=0&x1=999&y1=999&cleaning_parameter_set=1

#掃除停止
#/set/stop

#充電ドックへ戻る
#/set/go_home

#時刻設定？（1分置き位で呼ばれてる）
#/set/time?year=2018&month&=1&day=8&hour=20&min=3

#MAP系
#/get/feature_map
#/get/n_n_polygons
#/get/cleaning_grid_map
#/get/rob_pose

# HTTPリクエスト
url='http://192.168.1.23:10009/get/cleaning_grid_map'
req = requests.get(url)
dict = req.json()
lower_left_x = dict.get('lower_left_x')
lower_left_y = dict.get('lower_left_y')
size_x = dict.get('size_x')
size_y = dict.get('size_y')
resolution = dict.get('resolution')
cleaned = dict.get('cleaned')

# 地図を描画
plt.figure(figsize=(size_x * resolution / 100, size_y * resolution / 100))  # 解像度を考慮して描画サイズを設定

if (len(cleaned)>0):
    # cleanedは[1, a1, b1, a2, b2, .... ]となっており
    # 左下から右上にむかって、a1個：not cleaned、b1個；cleaned、と並んでいる
    map_data = [['cleaned' for _ in range(size_x)] for _ in range(size_y)]
    x, y = 0, 0

    # cleanedの情報を解釈して地図に反映
    for i in range(len(cleaned)):
        if i % 2 == 0:  # not cleanedの場合
            x += cleaned[i]
        else:  # cleanedの場合
            for _ in range(cleaned[i]):
                map_data[y][x] = 'not cleaned'
                x += 1
                if x == size_x:  # 右端に到達した場合は次の行へ
                    x = 0
                    y += 1
                    if y == size_y:  # 地図の範囲を超えた場合は終了
                        break

    for i in range(size_y):
        for j in range(size_x):
            if map_data[i][j] == 'cleaned':
                # x軸、y軸の座標を調整して描画
                plt.fill([lower_left_x + j * resolution, lower_left_x + (j+1) * resolution, lower_left_x + (j+1) * resolution, lower_left_x + j * resolution], 
                            [lower_left_y + i * resolution, lower_left_y + i * resolution, lower_left_y + (i+1) * resolution, lower_left_y + (i+1) * resolution], 
                            color='blue', alpha=0.5)  # 半透明に設定

plt.xticks(np.arange(lower_left_x, lower_left_x + (size_x + 1) * resolution, resolution))
plt.yticks(np.arange(lower_left_y, lower_left_y + (size_y + 1) * resolution, resolution))

# HTTPリクエスト
url='http://192.168.1.23:10009/get/feature_map'
req = requests.get(url)
dict = req.json()
map = dict.get('map')
lines = map.get('lines')
docking_pose = map.get('docking_pose')
n=len(lines)
print(lines)

#for文で部屋の地図情報を描画
for indx in range(0,n):
    x1=lines[indx].get('x1')
    y1=lines[indx].get('y1')
    x2=lines[indx].get('x2')
    y2=lines[indx].get('y2')
    plt.plot([x1, x2],[y1, y2], color='gray')
plt.plot([docking_pose.get('x')],[docking_pose.get('y')],marker="D",markersize=6) #ドック

# HTTPリクエスト
url='http://192.168.1.23:10009/get/rob_pose'
req = requests.get(url)
dict = req.json()
plt.plot([dict.get('x1'),dict.get('x2')],[dict.get('y1'),dict.get('y2')],marker="*",markersize=12) #イマココ

# HTTPリクエスト
url='http://192.168.1.23:10009/get/n_n_polygons'
req = requests.get(url)
dict = req.json()
map = dict.get('map')
polygons = map.get('polygons')

# for文
l=len(polygons)
for i in range(0,l):
    print(i)
    x=[]
    y=[]
    seg=polygons[i].get('segments')
    m=len(seg)
    for ii in range(0,m):
        print(ii)
        x1=seg[ii].get('x1')
        y1=seg[ii].get('y1')
        x2=seg[ii].get('x2')
        y2=seg[ii].get('y2')
        x=[x1,x2]
        print(x)
        y=[y1,y2]
        print(y)
        plt.plot(x,y,color='gray')

plt.grid()
plt.axis('equal')
plt.savefig('result.png')
plt.show()

# url='http://192.168.1.23:10009/set/clean_spot?map_id=0&x1=999&y1=999&cleaning_parameter_set=1'
# req = requests.get(url)
