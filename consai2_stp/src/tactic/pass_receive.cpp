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
// Created by hans on 2019/09/25.
//

#include <tactic/pass_receive.h>
#include <behavior_tree/sequence.h>
#include <behavior_tree/status_converter/always.h>
#include <tactic/skill/face_to_target.h>
#include <tactic/skill/go_point.h>
#include <tactic/skill/custom_skill.h>
#include <world_model/ball_info.h>

TacticPassReceive::TacticPassReceive(Point receive_point, float correction_enable_distance)
        : CORRECTION_ENABLE_DISTANCE(correction_enable_distance) {

    name = "PassReceive";
    this->receive_point = receive_point;

    addChild(CustomSkill::build([&](WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                                    std::shared_ptr<ControlTargetBuilder> builder) -> Status {
//        ROS_ERROR("receive_point : %f\t%f",receive_point.x(),receive_point.y());
        auto &ball = world_model.ball.pose;
        auto pos = robot->pose.pos;
        //こちらへ向かう速度成分
        Point target;
        float ball_vel = ball.vel.dot((pos - ball.pos).normalized());
        if (ball_vel < -0.5f) {
            world_model.has_play_finished = true;
            return Status::FAILURE;
        }
        if (ball_vel > 0.5f) {
            //ボールの進路上に移動
            ClosestPoint result;
            Segment ball_line(ball.pos, (ball.pos + ball.vel.normalized() * (ball.pos - pos).norm()));
            bg::closest_point(robot->pose.pos, ball_line, result);
            target = result.closest_point;
            builder->addTargetPose(target, tool::getAngle(ball.pos - pos));
        } else {
            //受け取り場所に移動
            builder->addTargetPose(this->receive_point, tool::getAngle(ball.pos - pos));
        }

        return Status::RUNNING;
    }));


    addChild(CustomSkill::build([this](WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                                       std::shared_ptr<ControlTargetBuilder> builder) -> Status {
        builder->addDribble(0.0f);
        return Status::RUNNING;
    }));
}

