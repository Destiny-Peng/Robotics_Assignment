# TurtleBot3 仿真环境搭建

**原项目仓库：** [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)

**部署过程参考：** [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

## 依赖安装

在 `/ros2` 路径下执行以下命令以下载并安装必要的依赖：

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

## 编译安装

执行下面命令以编译安装
```
source /opt/ros/humble/setup.bash
mkdir -p src
cd src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
sudo apt install python3-colcon-common-extensions
cd ..
colcon build --symlink-install
```
可以选择将环境变量设置写入bashrc中，也可以使用时再配置。
```
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 建图
启动gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

启动slam节点
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

启动遥控节点
ros2 run turtlebot3_teleop teleop_keyboard

观察建图效果，在建图完成后保存地图
ros2 run nav2_map_server map_saver_cli -f map

## 导航

启动gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

运行导航节点
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=map.yaml

在Rviz2中发布初始位姿来启动导航

启动遥控节点，通过小范围移动来优化初始姿态等
ros2 run turtlebot3_teleop teleop_keyboard

在Rviz2中发布Nav2 Goal,即可导航