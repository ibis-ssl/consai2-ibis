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
class TacticWallBallPlacement : public ParallelOne {
public:
    TacticWallBallPlacement(Point ball_pos) {
        //成功判定
        addChild(CustomSkill::build(
                [this](const WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) -> Status {
                    auto ball = world_model.ball.pose.pos;

                    //Lineから離れていれば完了
                    constexpr float LINE_THRESHOLD = 0.1f;  //TODO : 距離は適当なのでパラメータ化
                    float x_diff = (Constants::half_field_length() - abs(ball.x()));
                    float y_diff = (Constants::half_field_width() - abs(ball.y()));
                    if (x_diff > LINE_THRESHOLD && y_diff > LINE_THRESHOLD) {
                        ROS_INFO("BALL PLACEMENT FROM WALL SUCCESS");
                        return Status::SUCCESS;
                    }
                    return Status::RUNNING;
                }));

        float x_diff = (Constants::half_field_length() - abs(ball_pos.x()));
        float y_diff = (Constants::half_field_width() - abs(ball_pos.y()));

        Point target;
        constexpr float OFFSET = 0.3f;

        if (x_diff < y_diff) {
            target << std::copysign(1.f, -ball_pos.x()), 0.f;
            Eigen::Rotation2D<float> rot(M_PI / 6.f);
            target = rot * target;
            target.y() = std::copysign(target.y(), ball_pos.y());
        } else {
            target << 0.f, std::copysign(1.f, -ball_pos.y());
            Eigen::Rotation2D<float> rot(M_PI / 6.f);
            target = rot * target;
            target.x() = std::copysign(target.x(), ball_pos.x());
        }

        target = target.normalized() * OFFSET;
        target += ball_pos;
        Pose2D pose;
        pose.pos = target;
        pose.theta = tool::getAngle(ball_pos - target);

        auto wall_ball_placement = std::make_shared<Sequence>();
        wall_ball_placement->addChild(GoPoint::build(pose, 0.02f, 0.02f));
        auto kick = std::make_shared<ParallelOne>();
        kick->addChild(Kick::build());
        pose.pos = ball_pos + (ball_pos - target).normalized() * 0.1f;
        kick->addChild(GoPoint::build(pose, 0.03f, 0.05f));

        wall_ball_placement->addChild(kick);
        addChild(wall_ball_placement);

    }

    static std::shared_ptr<TacticWallBallPlacement> build(Point ball_pos) {
        return std::make_shared<TacticWallBallPlacement>(ball_pos);
    }

};
