

# nakbot_ros_sim package

nakbotのurdfモデルと、簡単なGazboシミュレーションのサンプルプログラム



## Build

```sh
git clone https://github.com/sskitajima/nakbot_ros_sim.git
cd $HOME/catkin_ws
catkin_make
```



## gazebo simulation

- 使い方

```sh
# rvizでロボットモデルを確認する
roslaunch nakbot_ros_sim display.launch
```



```sh
# gazeboにロボットを出現させる
roslaunch nakbot_ros_sim gazebo.launch
```



```sh
# 上の2つのファイルを同時に起動し、キーボードを用いてロボットを動かす
roslaunch nakbot_ros_sim nakbot_sample_simulator.launch X:=0.5 Y:=1 yaw:=3.14

# 任意のworldファイルでシミュレーションをしたいとき
roslaunch nakbot_ros_sim nakbot_sample_simulator.launch world:=$HOME/catkin_ws/src/nakbot_ros_sim/worlds/sample.world
```

## ros programming

- 基本(ros tutorial)

```sh
# publisher (src/listener.cpp)
rosrun nakbot_ros_sim nakbot_ros_sim_talker

# subscriber (src/talker.cpp)
rosrun nakbot_ros_sim nakbot_ros_sim_listener
```

- センサモデルのシミュレーション

```sh
# laserセンサを用いたサンプル(src/SD_sample_laser.cpp)
rosrun nakbot_ros_sim sample_laser_node

# cameraを用いたサンプル(src/SD_sample_camera.cpp)
rosrun nakbot_ros_sim sample_camera_node
```



## ファイルの説明

- ロボットモデルの定義...`urdf/`
  - ロボット全体...`my_robo.xacro`
  - LRF...`my_robo_laser.urdf.xacro`
  - camera...`my_robo_camera.urdf.xacro`
  - imu...`my_robo_imu.xacro`
- ロボットのコントローラ(差動二輪型モデルのシミュレーションを行うgazeboのプラグイン)...`config/controller.yaml`
