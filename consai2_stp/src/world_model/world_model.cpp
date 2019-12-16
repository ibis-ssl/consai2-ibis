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
// Created by hans on 2019/09/02.
//
#include <experimental/optional>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <std_msgs/String.h>
#include <consai2_msgs/BallInfo.h>
#include <consai2_msgs/RobotInfo.h>

#include <world_model/world_model.h>
#include <world_model/tool.h>

void WorldModel::update() {
    updateBallCloseRobots();
    //未実装
    updateEnemyThreat();

    field_analyzer.update(*this);
}

void WorldModel::resetControlTargetBuilder() {
    for (auto c : control_target_builder) {
        c->reset();
        c->resetPath();
    }
}

std::shared_ptr<RobotNode> TeamInfo::getRobot(uint8_t id) const {
    if (id < 0 || id >= Constants::max_id()) {
        return nullptr;
    }

    return robots.at(id);
}

WorldModel::WorldModel() {
    for (auto &&c : control_target_builder | boost::adaptors::indexed()) {
        c.value() = std::make_shared<ControlTargetBuilder>(c.index());
    }

    sub_geometry = nh.subscribe("/vision_receiver/raw_vision_geometry", 10, &WorldModel::geometryCallback, this);
    sub_decorded_ref = nh.subscribe("/referee_wrapper/decoded_referee", 10, &WorldModel::decodedRefereeCallback,
                                    this);
    sub_ball_info = nh.subscribe("/vision_wrapper/ball_info", 10, &WorldModel::ballInfoCallback, this);

    for (auto &&sub : subs_friend_info | boost::adaptors::indexed()) {
        std::stringstream ss;
        ss << "/vision_wrapper/robot_info_";
        if (friends.color == Color::YELLOW) {
            ss << "yellow_";
        } else {
            ss << "blue_";
        }
        ss << std::hex << sub.index();

        sub.value() = nh.subscribe(ss.str(), 10, &WorldModel::robotInfoCallback, this);
    }

    for (auto &&sub : subs_enemy_info | boost::adaptors::indexed()) {
        std::stringstream ss;
        ss << "/vision_wrapper/robot_info_";
        if (friends.color == Color::YELLOW) {
            ss << "blue_";
        } else {
            ss << "yellow_";
        }
        ss << std::hex << sub.index();

        sub.value() = nh.subscribe(ss.str(), 10, &WorldModel::robotInfoCallback, this);
    }

    pub_behavior_tree = nh.advertise<std_msgs::String>("/consai2_stp/behavior_tree", 10);

    for (auto &&pub : pubs_control_target | boost::adaptors::indexed()) {
        std::stringstream ss;
        ss << "/consai2_game/control_target_";
        if (friends.color == Color::YELLOW) {
            ss << "yellow_";
        } else {
            ss << "blue_";
        }
        ss << std::hex << pub.index();
        pub.value() = nh.advertise<consai2_msgs::ControlTarget>(ss.str(), 10);
    }

    for (auto &&robot: friends.robots | boost::adaptors::indexed()) {
        robot.value()->attribute.id = robot.index();
    }

    for (auto &&robot: enemys.robots | boost::adaptors::indexed()) {
        robot.value()->attribute.id = robot.index();
    }
}

void WorldModel::publishControlTargets() {
    for (auto &builder : this->control_target_builder) {
        auto robot = friends.getRobot(builder->getID());
        if (robot) {
            builder->generateAvoidingPoints(*this, robot);
            builder->addControlEnable();
            auto msg = builder->build(*this);
            pubs_control_target.at(builder->getID()).publish(msg);
        }
    }
}

void WorldModel::publishBehaviorTree() {
    auto robot = friends.getRobot(0);
    ptree tree;
    robot->behavior.serialize(tree);
    std::stringstream bt_stream;
    boost::property_tree::write_json(bt_stream, tree, true);

    std_msgs::String msg;
    msg.data = bt_stream.str();
    pub_behavior_tree.publish(msg);
}

void WorldModel::updateBallCloseRobots() {
    //uint8_t : index , float : squared distance from ball
    auto process = [=](const RobotArray &robots, RobotArray &output) {
        std::vector<std::pair<uint8_t, float>> ball_squared_distance;
        for (auto &&robot : robots | boost::adaptors::indexed()) {
            std::pair<uint8_t, float> info;
            info.first = robot.index();
            if (!robot.value()->is_disappeared) {
                info.second = bg::comparable_distance(robot.value()->pose.pos, ball.pose.pos);
            } else {
                info.second = 1000.0f;
            }
            ball_squared_distance.emplace_back(info);
        }

        //sort based on distance (second element of std::pair)
        std::sort(
                ball_squared_distance.begin(),
                ball_squared_distance.end(),
                [](const std::pair<uint8_t, float> &a, const std::pair<uint8_t, float> &b) {
                    return a.second < b.second;
                }
        );

        //距離順にoutputに出力
        for (auto id : ball_squared_distance | boost::adaptors::indexed()) {
            output.at(id.index()) = robots.at(id.value().first);
        }
    };

    process(friends.robots, ball_close_friends);
    process(enemys.robots, ball_close_enemy);

}

void WorldModel::updateEnemyThreat() {
    //TODO
}

void WorldModel::resetControlTarget() {
    //TODO
}

//TODO 現在は最小限の情報を取得しているだけなのでいろいろ頑張りたい
void WorldModel::ballInfoCallback(const consai2_msgs::BallInfo &msg) {
    ball.pose.pos << msg.pose.x, msg.pose.y;
    ball.pose.vel << msg.velocity.x, msg.velocity.y;

    ball.is_in_field = ball.isBallInField();
    ball.is_moved = ball.isBallMoved();
    ball.is_moving = ball.isBallMoving();

    Point ball_pos = ball.pose.pos;

    const auto &closest_friend = ball_close_friends.front();
    const auto &closest_enemy = ball_close_enemy.front();

    //敵ボールの時のキック判定(味方のキック判定は実際にキック命令が実行された時に更新)
    constexpr float SHOOTING_THRESHOLD = 1.0f;
    if (!ball.is_ours && ball.state == BallState::HOLD) {

        if (ball.pose.vel.norm() > SHOOTING_THRESHOLD) {
            ball.state = BallState::SHOOTING;
        }
    }

    if (ball.state == BallState::SHOOTING) {
        if (ball.pose.vel.norm() < SHOOTING_THRESHOLD) {
            ball.state = BallState::NONE;
        }
    }

    if (ball.state != BallState::SHOOTING) {
        //ボール所有者判定
        float friend_ball_dist = (closest_friend->pose.pos - ball_pos).norm();
        float enemy_ball_dist = (closest_enemy->pose.pos - ball_pos).norm();
//        ROS_WARN("E : %f\tM : %f", enemy_ball_dist, friend_ball_dist);
        if (friend_ball_dist < enemy_ball_dist) {
            ball.is_ours = true;
        } else {
            ball.is_ours = false;
        }

        //Hold判定
        constexpr float HOLD_THRESHOLD_DIST = 0.15f;
        constexpr float ANGLE_THRESHOLD = M_PI_4;
        if (ball.is_ours) {
            if (friend_ball_dist < HOLD_THRESHOLD_DIST) {
                float angle_diff = tool::getAngleDiff(closest_friend->pose.theta,
                                                      tool::getAngle(ball_pos - closest_friend->pose.pos));
                if (angle_diff < ANGLE_THRESHOLD) {
                    ball.state = BallState::HOLD;
                } else {
                    ball.state = BallState::NONE;
                }
            } else {
                ball.state = BallState::NONE;
            }
        } else {
            if (enemy_ball_dist < HOLD_THRESHOLD_DIST) {
                float angle_diff = tool::getAngleDiff(closest_enemy->pose.theta,
                                                      tool::getAngle(ball_pos - closest_enemy->pose.pos));
                if (angle_diff < ANGLE_THRESHOLD) {
                    ball.state = BallState::HOLD;
                } else {
                    ball.state = BallState::NONE;
                }
            } else {
                ball.state = BallState::NONE;
            }
        }
    }

    std::stringstream ss;
    if (ball.is_ours) {
        ss << "OUR ";
    } else {
        ss << "THEIR ";
    }
    switch (ball.state) {
        case BallState::NONE :
            ss << "NONE";
            break;
        case BallState::HOLD :
            ss << "HOLD";
            break;
        case BallState::SHOOTING:
            ss << "SHOOTING";
            break;
    }
//    ROS_INFO("%s", ss.str().c_str());
}

void WorldModel::robotInfoCallback(const ros::MessageEvent<consai2_msgs::RobotInfo const> &event) {
    has_robot_data_received = true;
    const ros::M_string &header = event.getConnectionHeader();
    std::string topic = header.at("topic");
    const consai2_msgs::RobotInfoConstPtr &msg = event.getMessage();

    Color color;
    if (topic.find("yellow") == std::string::npos) {
        //not found -> blue
        color = Color::BLUE;
    } else {
        color = Color::YELLOW;
    }

    std::shared_ptr<RobotNode> robot;
    if (friends.color == color) {
        robot = friends.getRobot(msg->robot_id);
        if (!robot) {
            return;
        }
    } else {
        robot = enemys.getRobot(msg->robot_id);
        if (!robot) {
            return;
        }
    }

    robot->is_disappeared = msg->disappeared;
    robot->pose = tool::getPose2D(msg->pose);
    robot->vel = tool::getPose2D(msg->velocity);
}
