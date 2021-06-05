#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // scan
#include <geometry_msgs/Twist.h>   // velocity

// クラスLaserSampleNodeの定義
class LaserSampleNode
{
private:
  ros::NodeHandle nh;

  ros::Subscriber sub_laser;
  ros::Publisher pub_vel;

  // laser scanの測定値を保存する変数の定義
  sensor_msgs::LaserScan latest_scan;

  // 速度指令値を保存する変数の定義
  geometry_msgs::Twist cmd_msg;

  // 2Hzで制御ループを回す
  const int loop_rate = 2;

  // メッセージの初回受信を確認する変数
  bool is_recieved_scan = false;

  const double laser_thres = 1.0;

public:
  LaserSampleNode()
  {
    sub_laser = nh.subscribe("/scan", 10, &LaserSampleNode::laser_callback, this);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  // 速度指令値を保持する変数に値を代入する関数
  geometry_msgs::Twist create_vel_msg(const double vel, const double omega)
  {
    // http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = vel;
    cmd_msg.angular.z = omega;

    return cmd_msg;
  }

  // laserトピックを受けたときに実行される関数
  // 最新のスキャン情報を保存しておく
  void laser_callback(const sensor_msgs::LaserScanConstPtr &laser_msg)
  {
    latest_scan = *laser_msg;
    is_recieved_scan = true;
  }

  void mainloop()
  {
    ros::Rate r(loop_rate);
    int cnt = 0;

    while (ros::ok())
    {
      ros::spinOnce();

      // 回転判定してから一定回数は回転司令を与え続ける
      cnt++;
      if (cnt < 4)
      {
        pub_vel.publish(cmd_msg);
        continue;
      }

      // 値を取得しないままlatest_scan変数にアクセスするとエラーを起こすので判定
      if (!is_recieved_scan)
        continue;

      // 複数あるセンサの値のうち、中央にある値を取得する
      // http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
      const double center_value = latest_scan.ranges[latest_scan.ranges.size() / 2];

      // 左右-10deg ~ 10deg 、サンプル数21点の範囲のLRFのとき
      // latest_scan.ranges[0] → -10degの方向にある距離
      // latest_scan.ranges[10]　→　0deg
      // latest_scan.ranges[20] →　10deg

      double min_distance = 99999;
      for (int i = 0; i < latest_scan.ranges.size(); i++)
      {
        double value = latest_scan.ranges[i];
        if (value < min_distance)
        {
          min_distance = value;
        }
      }

      if (min_distance < laser_thres)
      {
        cmd_msg = create_vel_msg(0, 0.3);
        cnt++;
      }
      else
      {
        cmd_msg = create_vel_msg(0.5, 0);
        cnt=0;
      }
      pub_vel.publish(cmd_msg);
      ROS_INFO("publish cmd_vel. linear velocity: %.2f, angular velocity: %.2f", cmd_msg.linear.x, cmd_msg.angular.z);

      // 指定したループ周期になるように調整
      r.sleep();
    }
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "laser_sample_node");

  // クラスLaserSampleNodeの実体となる変数nodeの定義
  LaserSampleNode node = LaserSampleNode();

  // メインループを回す
  node.mainloop();

  return 0;
}