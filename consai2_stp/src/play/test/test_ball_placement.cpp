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
#include <play/test/test_ball_placement.h>
#include <world_model/assignment.h>
#include <tactic/wall_ball_placement.h>
#include <tactic/ball_placement.h>


bool TestBallPlacement::initialize(WorldModel &world_model, TeamInfo &friends) {

    resetAssignment(world_model);

    auto robot = popBallClosest(world_model,false);
    if(!robot){
        return false;
    }

    //Line際からの脱出
    Point ball = world_model.ball.pose.pos;
    robot->behavior.addChild(TacticWallBallPlacement::build(ball));
    robot->behavior.addChild(TacticBallPlacement::build(world_model.ball_placement_target));
    return true;
}

bool TestBallPlacement::hasFinished(const WorldModel &world_model) {
    return false;
}

