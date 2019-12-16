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
#include <tactic/skill/spin_at_target.h>
#include <world_model/tool.h>

Status SpinAtTarget::run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                         std::shared_ptr<ControlTargetBuilder> builder) {
    ROS_INFO("SpinAtTarget");
    //angle from target to robot
    float now_angle = tool::getAngle(robot->pose.pos - target.getPoint(world_model));

    float target_angle = tool::getAngle(target.getPoint(world_model) - over_target.getPoint(world_model));
    if (tool::getAngleDiff(target_angle, now_angle) < TARGET_THRESHOLD) {
        return Status::SUCCESS;
    }

    float next_target_angle = target_angle;
    while (tool::getAngleDiff(next_target_angle, now_angle) >= M_PI_4) {
        next_target_angle = tool::getIntermediateAngle(next_target_angle, now_angle);
    }

    //make next target
    Pose2D next_target;
    float dist = (target.getPoint(world_model) - robot->pose.pos).norm();
    next_target.pos =
            target.getPoint(world_model) + Point(cos(next_target_angle), sin(next_target_angle)) * dist;
    next_target.theta = next_target_angle + M_PI;
    builder->addTargetPose(tool::getPose2D(next_target));

    return Status::RUNNING;
}
