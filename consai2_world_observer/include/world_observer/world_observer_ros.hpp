// MIT License
//
// Copyright (c) 2019 SSL-Roots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include <consai2_msgs/VisionDetections.h>
#include <consai2_msgs/RobotInfo.h>
#include <consai2_msgs/BallInfo.h>

#include <world_observer/geometry/geometry.hpp>

// InfoBaseクラス
// consai2_msgs/RobotInfo　及び consai2_msgs/BallInfoに対応するクラスのベースクラス
class InfoBase
{
public:
    InfoBase(geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp);

    geometry2d::Odometry odom_;
    bool detected_;
    bool disappeared_;
    geometry2d::Pose last_detection_pose_;
    ros::Time detection_stamp_;
};

// RobotInfoクラス
// consai2_msgs/RobotInfoに対応するクラス
class RobotInfo : public InfoBase
{
public:
    RobotInfo(int robot_id, geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp);
    consai2_msgs::RobotInfo ToROSMsg();

private:
    int robot_id_;
};

// BallInfoクラス
// consai2_msgs/BallInfoに対応するクラス
class BallInfo : public InfoBase
{
public:
    BallInfo(geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp);
    consai2_msgs::BallInfo ToROSMsg();
};

// ObservationContainer
// 一回のVisionによる観測値を格納しておくクラス
class ObservationContainer
{
public:
    std::vector<std::vector<geometry2d::Pose>> blue_observations;
    std::vector<std::vector<geometry2d::Pose>> yellow_observations;
    std::vector<geometry2d::Pose>                ball_observations;

    ObservationContainer(int num_of_robot);
};

// WorldObserverROS
// ROSとのIFクラス SubsciberやPublisherはここにまとめる
// ここを境目にROSのメッセージ型と内部で扱う型を分離する
class WorldObserverROS
{
public:
    int max_id;

    WorldObserverROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::string vision_topic_name);
    void VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg);
    bool RegisterUpdateHook(std::function<void(ObservationContainer observation_container)> function);

    void PublishBlueInfo(int robot_id, RobotInfo info);
    void PublishYellowInfo(int robot_id, RobotInfo info);
    void PublishBallInfo(BallInfo info);

    ~WorldObserverROS();

private:
    ros::Subscriber sub_vision_;
    ros::Publisher  pub_ball_info_;
    std::vector<ros::Publisher*> p_pub_blue_info_;
    std::vector<ros::Publisher*> p_pub_yellow_info_;
    std::function<void(ObservationContainer observation_container)> update_hook_;
};



void VisionCallbackHook(const WorldObserverROS& world_observer);
