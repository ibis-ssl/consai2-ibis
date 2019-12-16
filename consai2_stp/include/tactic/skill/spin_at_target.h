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
// Created by hans on 2019/09/07.
//

#pragma once

//-------------include----------------//
#include <tactic/skill/skill.h>
#include <utils/target.h>

//------------namespace---------------//
//--------------class-----------------//
class SpinAtTarget : public Skill {
protected:
    TargetModule target;
    TargetModule over_target;
    const float TARGET_THRESHOLD;
public:
    SpinAtTarget(TargetModule target, TargetModule over_target, float angle_threshold) : Skill("SpinAtTarget"),
                                                                                         target(target),
                                                                                         over_target(over_target),
                                                                                         TARGET_THRESHOLD(
                                                                                                 angle_threshold) {}

    static auto build(TargetModule target, TargetModule over_target, float angle_threshold = 0.1f) {
        return std::make_shared<SpinAtTarget>(target, over_target, angle_threshold);
    }

    virtual Status run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) override ;
};
