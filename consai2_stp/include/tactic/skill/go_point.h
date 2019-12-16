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
// Created by hans on 2019/09/03.
//

#pragma once

//-------------include----------------//
#include <tactic/skill/skill.h>
#include <utils/target.h>

//------------namespace---------------//
//--------------class-----------------//
class GoPoint : public Skill {
    enum class Mode {
        POSITION,
        POSE,
        TARGET,
        TARGET_LINE,
    } mode;
    TargetModule target;
    TargetLineModule target_ball_line;
    Pose2D target_pose;
    const float THRESHOLD_DIST;
    const float THRESHOLD_ANGLE;
public:
    GoPoint(TargetModule target_module, float threshold_dist) : Skill("GoPoint"), target(target_module),
                                                                THRESHOLD_DIST(threshold_dist),
                                                                THRESHOLD_ANGLE(M_PI_2) {
        mode = Mode::TARGET;
    }

    static auto build(TargetModule target_module, float threshold_dist) {
        return std::make_shared<GoPoint>(target_module, threshold_dist);
    }

    GoPoint(Point pos, float threshold_dist) : Skill("GoPoint"), THRESHOLD_DIST(threshold_dist),
                                               THRESHOLD_ANGLE(M_PI_2) {
        target_pose.pos = pos;
        mode = Mode::POSITION;
    }

    static auto build(Point pos, float threshold_dist) {
        return std::make_shared<GoPoint>(pos, threshold_dist);
    }

    GoPoint(Pose2D pose, float threshold_dist, float threshold_angle) : Skill("GoPoint"),
                                                                        THRESHOLD_DIST(threshold_dist),
                                                                        THRESHOLD_ANGLE(threshold_angle) {
        target_pose = pose;
        mode = Mode::POSE;
    }

    static auto build(Pose2D pose, float threshold_dist, float threshold_angle) {
        return std::make_shared<GoPoint>(pose, threshold_dist, threshold_angle);
    }

    GoPoint(TargetLineModule target_line, float threshold_dist) : Skill("GoPoint"), target_ball_line(target_line),
                                                                  THRESHOLD_DIST(threshold_dist),
                                                                  THRESHOLD_ANGLE(M_PI_2) {
        mode = Mode::TARGET_LINE;
    }

    static auto build(TargetLineModule target_line, float threshold_dist) {
        return std::make_shared<GoPoint>(target_line, threshold_dist);
    }

    virtual Status run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) override;
};
