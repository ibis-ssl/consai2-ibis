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
// Created by hans on 2019/08/24.
//

#pragma once

//-------------include----------------//
#include <tactic/skill/skill.h>
#include <world_model/tool.h>
#include <utils/target.h>
//--------------class-----------------//


class FaceToTarget : public Skill {
protected:
    const float ANGLE_THRESHOLD;
    TargetModule target;
public:
    /**
     * コンストラクタ
     * @param my_id　ロボットのID
     * @param angle_threshold targetの方を向いたと判定するしきい値(単位は度)
     */
    explicit FaceToTarget(TargetModule target, float angle_threshold = tool::getRad(2.0f)) : Skill("FaceToTarget"),
                                                                                             ANGLE_THRESHOLD(
                                                                                                     angle_threshold),
                                                                                             target(target) {}

    static auto build(TargetModule target, float angle_threshold = tool::getRad(2.0f)) {
        return std::make_shared<FaceToTarget>(target, angle_threshold);
    }

    virtual Status run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) override;
};

class FaceToBall : public FaceToTarget {
public:
    static std::shared_ptr<Component> build(float angle_threshold = tool::getRad(2.0f)) {
        return FaceToTarget::build(TargetModule::buildBall(), angle_threshold);
    }
};
