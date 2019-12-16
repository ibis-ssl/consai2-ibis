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
// Created by hans on 2019/11/28.
//

#pragma once

//-------------include----------------//
#include <tactic/skill/skill.h>

//------------namespace---------------//
//--------------class-----------------//
class Kick : public Skill {
protected:
    const float KICK_POWER;
public:
    explicit Kick(float kick_power) : Skill("Kick"), KICK_POWER(kick_power) {}

    static auto build(float kick_power = 1.f) {
        return std::make_shared<Kick>(kick_power);
    }

    virtual Status run(WorldModel &world_model, const std::shared_ptr<RobotNode> robot,
                       std::shared_ptr<ControlTargetBuilder> builder) override {
        builder->addKick(KICK_POWER);
        return Status::RUNNING;
    }
};

