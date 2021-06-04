

# nakbot_ros_sim package

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
roslaunch nakbot_ros_sim nakbot_empty_simulator.launch X:=0.5 Y:=1 yaw:=3.14

# 任意のworldファイルでシミュレーションをしたいとき
roslaunch nakbot_ros_sim nakbot_sample_simulator.launch world:=$HOME/catkin_ws/src/nakbot_ros_sim/worlds/sample.world
```

## ros programming

- 基本(ros tutorial)

```sh
# publisher
rosrun nakbot_ros_sim nakbot_ros_sim_talker

# subscriber
rosrun nakbot_ros_sim nakbot_ros_sim_listener
```

- センサモデルのシミュレーション

```sh
# laser
rosrun nakbot_ros_sim sample_laser_node

# camera
rosrun nakbot_ros_sim sample_camera_node
```



## ファイルの説明

- ロボットモデルの定義...`urdf/`
- ロボットのコントローラ(差動二輪型モデルのシミュレーションを行うgazeboのプラグイン)...`config/controller.yaml`
