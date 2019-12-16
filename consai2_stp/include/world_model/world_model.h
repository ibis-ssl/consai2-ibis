// MIT License
//
// Copyright (c) 2019 ibis-ssl
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
//
// Created by hans on 2019/08/17.
//

#pragma once

//-------------include----------------//
#include <memory> //std::shareed_ptr
#include <deque>
#include <string>
#include <experimental/optional>

#include <boost/range/adaptor/indexed.hpp>
#include <ros/ros.h>
#include <consai2_msgs/DecodedReferee.h>

#include <utils/control_target_builder.h>
#include <world_model/robot_node.h>
#include <world_model/ball_info.h>
#include <world_model/constants.h>
#include <field_analysis/field_analyzer.h>

//------------namespace---------------//
//--------------class-----------------//

namespace consai2_msgs {
    template<class ContainerAllocator>
    struct VisionGeometry_;
    template<class ContainerAllocator>
    struct BallInfo_;
    template<class ContainerAllocator>
    struct RobotInfo_;

    typedef ::consai2_msgs::VisionGeometry_<std::allocator<void> > VisionGeometry;
    typedef ::consai2_msgs::BallInfo_<std::allocator<void> > BallInfo;
    typedef ::consai2_msgs::RobotInfo_<std::allocator<void> > RobotInfo;
}

class RobotNode;

using RobotArray = std::array<std::shared_ptr<RobotNode>, Constants::max_id()>;

enum class Color {
    BLUE,
    YELLOW
};

struct TeamInfo {
    TeamInfo() {
        for (auto &robot : robots) {
            robot = std::make_shared<RobotNode>();
        }
    }

    uint8_t goalie_id;
    Color color;
    RobotArray robots;

    std::shared_ptr<RobotNode> getRobot(uint8_t id) const;

    auto getGoalie() const {
        return getRobot(goalie_id);
    }
};


class WorldModel {
protected:
    ros::NodeHandle nh;
    ros::Subscriber sub_geometry;
    ros::Subscriber sub_decorded_ref;
    ros::Subscriber sub_ball_info;
    std::array<ros::Subscriber, Constants::max_id()> subs_friend_info;
    std::array<ros::Subscriber, Constants::max_id()> subs_enemy_info;

    consai2_msgs::DecodedReferee decoded_referee;

public:

    volatile bool has_robot_data_received = false;
    volatile bool has_geometry_data_received = false;
    volatile bool has_initialized = false;
    bool has_play_finished = true;

    BallInfo ball;
    TeamInfo friends;
    TeamInfo enemys;

    //threat based defence用
    //TODO : 活用
    RobotArray enemy_threat;

    RobotArray ball_close_friends;
    RobotArray ball_close_enemy;

    std::array<std::shared_ptr<ControlTargetBuilder>, Constants::max_id()> control_target_builder;
    std::array<ros::Publisher, Constants::max_id()> pubs_control_target;
    ros::Publisher pub_behavior_tree;

    FieldAnalyzer field_analyzer;

    Point ball_placement_target;

    void update();

    void resetControlTargetBuilder();

public:
    WorldModel();

    void publishControlTargets();

    void publishBehaviorTree();

    bool isAvailable() {
        return has_robot_data_received && has_geometry_data_received;
    }

    auto getDecodedReferee() { return decoded_referee; }

private:

    void updateBallCloseRobots();

    void updateEnemyThreat();

    void resetControlTarget();


    void geometryCallback(const consai2_msgs::VisionGeometry &msg) {
        has_geometry_data_received = true;
        Constants::update(msg);
    }

    void decodedRefereeCallback(const consai2_msgs::DecodedReferee &msg) {
        decoded_referee = msg;
    }

    void ballInfoCallback(const consai2_msgs::BallInfo &msg);

    void robotInfoCallback(const ros::MessageEvent<consai2_msgs::RobotInfo const> &event);
};
