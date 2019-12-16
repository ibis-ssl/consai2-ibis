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
#include <tactic/skill/face_to_target.h>

Status FaceToTarget::run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                         std::shared_ptr<ControlTargetBuilder> builder) {

    auto target_pos = target.getPoint(world_model);
    auto my_pos = robot->pose.pos;

    float target_angle = tool::getAngle(target_pos - my_pos);

    //目標値との角度差がしきい値未満のとき成功判定
    if (tool::getAngleDiff(robot->pose.theta, target_angle) < ANGLE_THRESHOLD) {
        return Status::SUCCESS;
    }

    auto target = builder->getFrontTarget();
    //移動先が存在するなら目標角を書き換え
    if (target) {
        builder->setTargetTheta(target_angle);
    } else {//移動先が存在しないなら新規作成
        builder->addTargetPose(my_pos, target_angle);
    }
    return Status::RUNNING;
}
