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
// Created by hans on 2019/08/24.
//
#include <utils/control_target_builder.h>
#include <world_model/world_model.h>
#include <world_model/robot_node.h>
#include <world_model/constants.h>
#include <world_model/tool.h>


ControlTargetBuilder &
ControlTargetBuilder::addTargetPose(Point pos, float theta, float threshold_dist, float threshold_angle) {
    //しきい値より近い場合，目標点の挿入を行わない
    auto front = getFrontTarget();
    if (front) {
        auto front_pos = tool::getPoint(*front);
        auto front_angle = cmd.path.front().theta;
        if ((front_pos - pos).norm() < threshold_dist) {
            if (abs(front_angle - theta) < threshold_angle) {
                return *this;
            }
        }
    }

    geometry_msgs::Pose2D pose;
    pose.x = pos.x();
    pose.y = pos.y();
    pose.theta = theta;
    cmd.path.emplace_back(pose);
    return *this;
}

ControlTargetBuilder &
ControlTargetBuilder::addTargetPose(geometry_msgs::Pose2D pose, float threshold_dist, float threshold_angle) {
    return addTargetPose(tool::getPoint(pose), pose.theta, threshold_dist, threshold_angle);
}

Obstacles
ControlTargetBuilder::getObstaclesDistanceFromSegment(const Point &p1, const Point &p2,
                                                      const Obstacles &obstacles, const float THRESHOLD_DISTANCE) {
    Point segment = p2 - p1;
    Obstacles detected;
    for (auto obs : obstacles) {

        //p1より後ろに居たら無視
        if (segment.dot(obs.center - p1) < 0) {
            continue;
        }
        //p2より向こうに居たら無視
        if (segment.dot(obs.center - p2) > 0) {
            continue;
        }
        Obstacle info;
        info.center = obs.center;

        Segment seg(p1, p2);
        info.radius = bg::distance(seg, obs.center);
        if (info.radius < THRESHOLD_DISTANCE) {
            detected.emplace_back(info);
        }
    }
    return detected;
}

Obstacles
ControlTargetBuilder::getObstaclesDistanceFromPoint(const Point &point,
                                                    const Obstacles &obstacles) {
    Obstacles detected;
    for (auto obs : obstacles) {
        Obstacle info;
        info.center = obs.center;
        info.radius = bg::distance(point, obs.center);
        detected.emplace_back(info);
    }
    return detected;
}

void ControlTargetBuilder::generateAvoidingPoints(const WorldModel &world_model, std::shared_ptr<RobotNode> robot) {
    Obstacles obstacles;
    if (avoid_enemy) {
        addEnemyToObstacle(world_model, obstacles);
    }
    if (avoid_friend) {
        addFriendToObstacle(world_model, obstacles);
    }
    if (avoid_ball) {
        addBallToObstacle(world_model, obstacles);
    }
    if (avoid_defence_area) {
        //TODO
    }

    //判定
    if (cmd.path.size() > 0) {
        Point target = tool::getPoint(cmd.path.at(0));
        //障害物とpathの距離を計算
        auto obstacles_dist = getObstaclesDistanceFromSegment(robot->pose.pos, target, obstacles, 0.5f);

        //回避点生成

        if (avoid_point) {
            auto p = tool::getPose2D(*avoid_point);
            //角度は補間
            p.theta = cmd.path.front().theta;
            cmd.path.insert(cmd.path.begin(), p);
        }

        auto tmp_avoid_point = generateAvoidPoint(world_model, obstacles_dist, robot);

        if (tmp_avoid_point) {

            //目標が生成した回避点よりも近い場合，回避点を使わない
            constexpr float HYSTERESIS = 4.0f;
            if (bg::comparable_distance(*tmp_avoid_point, robot->pose.pos) + HYSTERESIS >
                bg::comparable_distance(target, robot->pose.pos)) {
                tmp_avoid_point = std::experimental::nullopt;
            }
        }

        if (avoid_point) {
            if (tmp_avoid_point) {
                auto p = tool::getPose2D(*tmp_avoid_point);
                p.theta = cmd.path.front().theta;
                cmd.path.front() = p;
                avoid_point = tmp_avoid_point;
            }
        } else {
            if (tmp_avoid_point) {
                auto p = tool::getPose2D(*tmp_avoid_point);
                p.theta = cmd.path.front().theta;
                cmd.path.insert(cmd.path.begin(), p);
                avoid_point = tmp_avoid_point;
            }
        }
    }
}

std::experimental::optional<Point>
ControlTargetBuilder::generateAvoidPoint(const WorldModel &world_model, const Obstacles &obst_dist,
                                         std::shared_ptr<RobotNode> robot) {
    if (obst_dist.size() > 0) {
        //一番近い障害物
        auto closest_obs = std::min_element(obst_dist.begin(), obst_dist.end(),
                                            [robot](Obstacle a, Obstacle b) -> bool {
                                                return bg::comparable_distance(a.center, robot->pose.pos) <
                                                       bg::comparable_distance(b.center, robot->pose.pos);
                                            });
        auto my_pose = robot->pose;

        //TODO : param化
        constexpr float OFFSET_VERTICAL = 0.0;
        constexpr float OFFSET_HORIZONTAL = 0.5;

        //障害物の左右に回避点を生成
        Point diff = (closest_obs->center - my_pose.pos).normalized();
        Point offset_vertical = diff * OFFSET_VERTICAL;
        Point offset_horizontal = tool::getVerticalVec(diff) * OFFSET_HORIZONTAL;

        Point avoid_point_1 = closest_obs->center + offset_vertical + offset_horizontal;
        Point avoid_point_2 = closest_obs->center + offset_vertical - offset_horizontal;

        auto target = tool::getPoint(cmd.path.front());
        //近い方の回避点を採用
        Point avoid_point;
//        constexpr float HYSTERESIS = 0.001f;
        if (bg::comparable_distance(target, avoid_point_1) <
            bg::comparable_distance(target, avoid_point_2)) {
            avoid_point = avoid_point_1;
        } else {
            avoid_point = avoid_point_2;
        }

        return std::experimental::make_optional(avoid_point);
    }
    return std::experimental::nullopt;
}


void ControlTargetBuilder::addEnemyToObstacle(const WorldModel &world_model, Obstacles &obstacles) {
    const auto &robots = world_model.enemys.robots;
    float radius = Constants::robot_radius() + getEnemyAvoidanceOffset();
    for (auto r : robots) {
        if (r->is_disappeared) {
            continue;
        }
        Obstacle obs;
        obs.radius = radius;
        obs.center = r->pose.pos;
        obstacles.emplace_back(obs);
    }
}

void ControlTargetBuilder::addFriendToObstacle(const WorldModel &world_model, Obstacles &obstacles) {
    const auto &robots = world_model.friends.robots;
    float radius = Constants::robot_radius() + getFriendAvoidanceOffset();
    for (auto r : robots) {
        if (r->is_disappeared) {
            continue;
        }
        if (r->attribute.id == cmd.robot_id) {
            continue;
        }
        Obstacle obs;
        obs.radius = radius;
        obs.center = r->pose.pos;
        obstacles.emplace_back(obs);
    }
}

void ControlTargetBuilder::addBallToObstacle(const WorldModel &world_model, Obstacles &obstacles) {
    Obstacle obs;
    obs.radius = Constants::ball_radius() + getBallAvoidanceOffset();
    obs.center = world_model.ball.pose.pos;
    obstacles.emplace_back(obs);
}

consai2_msgs::ControlTarget ControlTargetBuilder::build(const WorldModel &world_model) {
    cmd.ball_x = world_model.ball.pose.pos.x();
    cmd.ball_y = world_model.ball.pose.pos.y();
    //TODO
//        generateAvoidingPoints(world_model,)
//        auto avoid_point = generateAvoidPoint(world_model,);
    return cmd;
}





