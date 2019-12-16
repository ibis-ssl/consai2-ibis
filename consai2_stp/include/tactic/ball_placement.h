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
// Created by hans on 2019/11/28.
//

#pragma once

//-------------include----------------//
#include <behavior_tree/parallel_all.h>
#include <tactic/skill/custom_skill.h>
#include <world_model/ball_info.h>
#include <tactic/skill/go_point.h>
#include <behavior_tree/parallel_one.h>
#include <world_model/tool.h>
#include <tactic/skill/kick.h>
#include <Eigen/Geometry>

//------------namespace---------------//
//--------------class-----------------//
class TacticBallPlacement : public ParallelOne {
private:
    Point placement_pos;
public:
    TacticBallPlacement(Point placement_pos) : placement_pos(placement_pos) {
        //成功判定
        addChild(CustomSkill::build(
                [this](const WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) -> Status {
                    auto ball = world_model.ball.pose.pos;

                    float dist = bg::distance(ball, this->placement_pos);
                    if (dist < 0.02f) {
                        ROS_INFO("BALL PLACEMENT  SUCCESS");
                        return Status::SUCCESS;
                    }
                    return Status::RUNNING;
                }));
        addChild(CustomSkill::build(
                [this](const WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) -> Status {

                    auto ball = world_model.ball.pose.pos;
                    auto pos = robot->pose.pos;
                    auto target = this->placement_pos;

                    //機体がボールの後ろにない場合，ボールの後ろに移動
                    float dot = (ball - pos).normalized().dot((target - ball).normalized());
                    if (dot < 0.95f) {
                        Point ball_back = ball + (ball - target).normalized() * 0.3f;
                        builder->resetPath();
                        builder->addTargetPose(ball_back, tool::getAngle(target - ball), 0.02f, 0.02f);

                        return Status::RUNNING;
                    }

                    //ボールを目標に向けてドリブル
                    Point dribble_target = ball + (target - ball).normalized() * 0.05f;
                    builder->resetPath();
                    builder->addTargetPose(dribble_target, tool::getAngle(target - ball), 0.02f, 0.02f).addDribble(
                            0.5f);
                    return Status::RUNNING;

                }));

    }

    static std::shared_ptr<TacticBallPlacement> build(Point placement_pos) {
        return std::make_shared<TacticBallPlacement>(placement_pos);
    }

};

