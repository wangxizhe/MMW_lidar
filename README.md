# MMW_lidar
A project to draw a marker line

## 安装方法  
mkdir -p ~/catkin_workspace/src  
cd catkin_workspace/src  
git clone https://github.com/wangxizhe/MMW_lidar.git  
cd ..  
catkin_make  

## 运行方法  
source devel/setup.bash  
roslaunch MMW_lidar MMW_lidar.launch  
打开新的终端运行  
cd catkin_workspace/src/MMW_lidar/include  
rosbag play test.bag  
即可在Rviz界面观察到标记线
