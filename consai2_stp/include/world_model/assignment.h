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
// Created by hans on 2019/11/21.
//

#pragma once

//-------------include----------------//
#include <memory>
#include <utils/boost_geometry.h>

//------------namespace---------------//
//--------------class-----------------//
class WorldModel;

class RobotNode;

void resetAssignment(WorldModel &world_model, bool preserve_goalie = false);

std::shared_ptr<RobotNode> popBallClosest(WorldModel &world_model, bool avoid_goalie);

std::shared_ptr<RobotNode>
popMaxPassRate(WorldModel &world_model, Point &receive_point, bool avoid_goalie);

std::shared_ptr<RobotNode> popMaxTotalRate(WorldModel &world_model, Point &receive_point, bool avoid_goalie);

std::shared_ptr<RobotNode> popGoalie(WorldModel &world_model);

std::vector<std::shared_ptr<RobotNode>> popRestRobots(WorldModel &world_model, bool avoid_goalie = false);
