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
// Created by hans on 2019/10/19.
//

#pragma once

//-------------include----------------//
#include <play/play.h>
//#include <world_model/ball_info.h>
#include <tactic/skill/go_point.h>

//------------namespace---------------//
//--------------class-----------------//
class OurKickOffPreparation : public RefereePlay {
public:
    OurKickOffPreparation() : RefereePlay("OUR_KICKOFF_PREPARATION") {}

    virtual std::vector<GameSituation> getTriggers() const override {
        return {GameSituation::OUR_KICKOFF_PREPARATION};
    }

    virtual void initialize(const WorldModel &world_model, TeamInfo &friends) override {
        auto ball = world_model.ball.pose.pos;
        Point target = ball + (ball -  Constants::Our::Goal::center()).normalized()*0.3f;
        world_model.ball_close_friends.front()->behavior.addChild(GoPoint::build(target,0.1f));
    }
};
