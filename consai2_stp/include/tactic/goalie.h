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
// Created by hans on 2019/08/19.
//

#pragma once

//-------------include----------------//
#include <tactic/skill/face_to_target.h>
#include <behavior_tree/parallel_all.h>
#include <tactic/skill/custom_skill.h>
#include <world_model/ball_info.h>

//------------namespace---------------//
//--------------class-----------------//
class TacticGoalie : public ParallelAll {
public:
    TacticGoalie() {
        addChild(CustomSkill::build(
                [this](const WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) -> Status {
                    auto ball_pos = world_model.ball.pose.pos;
                    auto pos = robot->pose.pos;
//                    auto goal_pos = Constants::Our::Goal::center();
                    //ゴールエリア内にボールがある場合，外に向けて蹴る(TODO : この分岐はConditionで行いたい)
                    if(tool::isInOurDefenseArea(ball_pos) && world_model.ball.pose.vel.norm() < 0.1f){
                        //TODO : 現在は適当に蹴ってるだけなので考えて蹴る
                        Point target(0,0);
                        //機体がボールの後ろにない場合，ボールの後ろに移動
                        float dot = (ball_pos - pos).normalized().dot((target - ball_pos).normalized());
                        if (dot < 0.95f) {
                            Point ball_back = ball_pos + (ball_pos - target).normalized() * 0.3f;
                            builder->resetPath();
                            builder->addTargetPose(ball_back, tool::getAngle(target - ball_pos), 0.02f, 0.02f);

                            return Status::RUNNING;
                        }

                        //ボールを目標に向けてキック
                        Point kick_target = ball_pos + (target - ball_pos).normalized() * 0.05f;
                        builder->resetPath();
                        builder->addTargetPose(kick_target, tool::getAngle(target - ball_pos), 0.02f, 0.02f).addKick(
                                1.f);
                        return Status::RUNNING;
                    }
                    //ボール速度線がゴールに向かっていた場合，速度線上に移動して防衛
                    std::vector<Point> vel_intersection;
                    Segment ball_vel_line(ball_pos, ball_pos + world_model.ball.pose.vel.normalized() * 30.f);
                    Segment goal_line(Constants::Our::Goal::lower(), Constants::Our::Goal::upper());
                    bg::intersection(ball_vel_line, goal_line, vel_intersection);
                    if (!vel_intersection.empty()) {
                        ROS_WARN("SHOOT");
                        ClosestPoint result;
                        bg::closest_point(pos, ball_vel_line, result);
                        builder->addTargetPose(result.closest_point, tool::getAngle(ball_pos - pos));
                        return Status::RUNNING;
                    }

                    //通常時はボールとゴールの間にいる
                    //ボールライン
                    Segment ball_line(Constants::Our::Goal::center(), ball_pos);
                    //前に出る距離
                    constexpr float OFFSET = 0.3f;
                    Point goal_center = Constants::Our::Goal::center();
                    goal_center.x() += OFFSET;


                    // 防衛位置を計算
                    // ボールのy座標によって防衛ラインを使い分ける
                    std::vector<Point> goal_intersection;
                    if (ball_pos.y() < 0.0f) {
                        Segment goal_line_lower(goal_center, Constants::Our::Goal::lower());
                        bg::intersection(ball_line, goal_line_lower, goal_intersection);
                    } else {
                        Segment goal_line_upper(goal_center, Constants::Our::Goal::upper());
                        bg::intersection(ball_line, goal_line_upper, goal_intersection);
                    }

                    //交点がなければゴールの真ん中
                    //あれば交点上に移動
                    Pose2D pose;
                    if (goal_intersection.empty()) {
                        pose.pos = Constants::Our::Goal::center();
                    } else {
                        pose.pos = goal_intersection.front();
                    }
                    builder->addTargetPose(pose.pos, tool::getAngle(ball_pos - pos));
                    return Status::RUNNING;
                }));

    }

    static std::shared_ptr<TacticGoalie> build() {
        return std::make_shared<TacticGoalie>();
    }

};
