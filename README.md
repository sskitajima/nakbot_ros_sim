

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
```

## ros programming

- 基本(ros tutorial)

```sh
# publisher
rosrun nakbot_ros_sim 

# subscriber

```



