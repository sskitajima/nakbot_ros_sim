#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

// global変数として定義（大きなプログラムではあまり良い書き方ではない）
std::string topic_name;

// メッセージを受け取った（subscribe）したときに実行される関数
void ImageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // opencvで画像を示す型を用意
    cv::Mat raw_img;

    const char *encoding = img_msg->encoding.c_str();

    ROS_INFO("topic %s, encoding %s", topic_name.c_str(), encoding);

    // 2つの文字列が等しいときに 0 を返す関数 strcmp
    // もし8bit rgbの画像だったとき、処理を行う
    if (!strcmp(encoding, "bgr8"))
    {
        try
        {
            // 画像をbgrの8bit形式に変換を試み、それを表示
            cv::imshow("view", cv_bridge::toCvShare(img_msg, "bgr8")->image);

            // 30ms表示
            cv::waitKey(30);
        }
        catch (cv_bridge::Exception &e)
        {
            // 変換に失敗すると、メッセージを出す
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
        }
    }
}

// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nakbot_ros_sim_image_subscriber");
    ROS_INFO("nakbot_ros_sim_image_subscriber node start");

    // ノードハンドラ
    ros::NodeHandle nh;

    nh.param("/nakbot_ros_sim_image_subscriber/topic_name", topic_name, std::string("default"));

    // サブスクライバの定義
    // サブスクライブしたときに実行される、関数を登録しておく。
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topic_name, 1, ImageCallback);

    // 処理をこの行でブロックし、サブスクライブされるのを待ち続ける
    ros::spin();

    return 0;
}