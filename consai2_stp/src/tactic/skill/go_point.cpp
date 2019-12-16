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
#include <tactic/skill/go_point.h>
#include <world_model/tool.h>

Status GoPoint::run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                    std::shared_ptr<ControlTargetBuilder> builder) {
//成功判定
    Point t_pos(0,0);
    float t_angle = 0;
    if (mode == Mode::TARGET) {
        t_pos = target.getPoint(world_model);
//        t_angle = target.getPose(world_model).theta;
    } else if (mode == Mode::TARGET_LINE) {
        ClosestPoint result;
        bg::closest_point(robot->pose.pos, target_ball_line.getLine(world_model), result);
        t_pos = result.closest_point;
        float dist = bg::distance(robot->pose.pos, result.closest_point);
        if (dist < THRESHOLD_DIST) {
            return Status::SUCCESS;
        }

    } else {
        t_pos = target_pose.pos;
        t_angle = target_pose.theta;
    }

    if (mode != Mode::TARGET_LINE && (t_pos - robot->pose.pos).norm() < THRESHOLD_DIST) {
        if (mode == Mode::POSITION) {
            return Status::SUCCESS;
        } else if (tool::getAngleDiff(robot->pose.theta, t_angle) < THRESHOLD_ANGLE) {
            return Status::SUCCESS;
        }
    }

//目標セット
    switch (mode) {
        case Mode::POSITION:
            target_pose.theta = robot->pose.theta;
            builder->addTargetPose(tool::getPose2D(target_pose), THRESHOLD_DIST, THRESHOLD_ANGLE);
            break;
        case Mode::POSE:
            builder->addTargetPose(tool::getPose2D(target_pose), THRESHOLD_DIST, THRESHOLD_ANGLE);
            break;
        case Mode::TARGET: {
            geometry_msgs::Pose2D pose = tool::getPose2D(target.getPoint(world_model));
            pose.theta = robot->pose.theta;
            builder->addTargetPose(pose, THRESHOLD_DIST, THRESHOLD_ANGLE);
            break;
        }
        case Mode::TARGET_LINE: {
            ClosestPoint result;
            bg::closest_point(robot->pose.pos, target_ball_line.getLine(world_model), result);
            geometry_msgs::Pose2D pose = tool::getPose2D(result.closest_point);
            pose.theta = robot->pose.theta;
            builder->addTargetPose(pose, THRESHOLD_DIST, THRESHOLD_ANGLE);
            break;
        }
    }
    return Status::RUNNING;
}
