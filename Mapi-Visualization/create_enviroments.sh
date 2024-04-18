pip3 install requests

cd Desktop/
mkdir -p ros2_ws/src
cd /ros2_ws/src

ros2 pkg create --build-type ament_python --node-name mapi mapi

cd ..
colcon build
source install/setup.bas
ros2 run my_package my_node
