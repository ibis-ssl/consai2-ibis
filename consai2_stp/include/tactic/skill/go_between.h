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
// Created by hans on 2019/08/31.
//

#pragma once

//-------------include----------------//
#include <tactic/skill/skill.h>
#include <utils/target.h>

//--------------class-----------------//
/**
 * target1とtarget2の間に移動する
 * パスの阻止などに使う
 */
class GoBetween : public Skill {
protected:
    TargetModule target1;
    TargetModule target2;
public:
    /**
     * コンストラクタ
     * @param my_id ロボットのID
     * @param t1 Target1
     * @param t2 Target2
     */
    GoBetween(TargetModule t1, TargetModule t2, float offset) : Skill("GoBetween") {
        target1 = t1;
        target2 = t2;
    }

    static auto build(TargetModule t1, TargetModule t2, float offset) {
        return std::make_shared<GoBetween>(t1, t2, offset);;
    }

    virtual Status run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) override;

};
