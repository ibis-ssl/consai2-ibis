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
#include <tactic/skill/go_between.h>
#include <world_model/tool.h>

Status GoBetween::run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                      std::shared_ptr<ControlTargetBuilder> builder) {

    Point my_pos = robot->pose.pos;
    Point t1 = target1.getPoint(world_model);
    Point t2 = target2.getPoint(world_model);
//2点間の線分の最近点を目標にする
    ClosestPoint result;
    bg::closest_point(my_pos,tool::getSegment(t1,t2),result);
    Point target_pos = result.closest_point;

//端っこが最近点である場合，真ん中を目標にする(端点はロボットだったりするので実際には行けないことが多い)
    if (target_pos == t1 || target_pos == t2) {
        target_pos = (t1 + t2) / 2.0f;
    }

    builder->addTargetPose(target_pos, robot->pose.theta);

    return Status::RUNNING;
}
