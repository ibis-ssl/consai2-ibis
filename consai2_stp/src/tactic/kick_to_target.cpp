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
#include <tactic/kick_to_target.h>
#include <behavior_tree/parallel_one.h>
#include <tactic/skill/custom_skill.h>
#include <world_model/tool.h>


TacticKickToTarget::TacticKickToTarget(TargetModule target, float power) : target(target), POWER(power) {
    name = "KickToTarget";
    {// go position near ball(30cm)
        auto go_near = std::make_shared<ParallelOne>();

        go_near->addChild(CustomSkill::build(
                [this](const WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) -> Status {
                    Point ball_pos = world_model.ball.pose.pos;
                    Point target = this->target.getPoint(world_model);

                    Pose2D pose;

                    pose.pos = ball_pos + (ball_pos - target).normalized() * 0.3f;
                    pose.theta = tool::getAngle(target - robot->pose.pos);

                    if (bg::distance(pose.pos, robot->pose.pos) < 0.05f && tool::getAngleDiff(pose.theta, robot->pose.theta) < 0.05f) {
                        ROS_INFO("APPROACH SUCCESS");
                        return Status::SUCCESS;
                    }

                    builder->addTargetPose(tool::getPose2D(pose), 0.02f, 0.01f);
                    return Status::RUNNING;
                }));
        // fix status
        addChild(go_near);

        {// kick
            addChild(CustomSkill::build(
                    [this](const WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                           std::shared_ptr<ControlTargetBuilder> builder) -> Status {
                        auto ball_pos = world_model.ball.pose.pos;
                        auto target = this->target.getPoint(world_model);
                        Pose2D pose;
                        //10cm行き過ぎる
                        pose.pos = ball_pos - (ball_pos - target).normalized() * 0.1f;
                        pose.theta = tool::getAngle(target - robot->pose.pos);

                        float diff = bg::distance(ball_pos, robot->pose.pos);
                        float diff_theta = tool::getAngleDiff(pose.theta, robot->pose.theta);
                        if (diff < 0.2 && diff_theta < 0.1) {
                            builder->addKick(POWER);
                        } else {
                            builder->addDribble(0.0);
                        }

                        if (world_model.ball.pose.vel.norm() > 1.0f) {
                            ROS_INFO("KICK SUCCESS");
                            return Status::SUCCESS;
                        }

                        builder->addNoAvoidanceBall().addNoAvoidanceRobot().addTargetPose(
                                tool::getPose2D(pose), 0.05f, 0.01f);

                        return Status::RUNNING;
                    }));
        }

    }
}
