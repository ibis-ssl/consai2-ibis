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
#include <world_model/assignment.h>
#include <world_model/world_model.h>

bool isValidRobot(std::shared_ptr<RobotNode> robot, bool avoid_goalie) {
    if (robot->is_disappeared) {
        return false;
    }
    if (robot->attribute.is_assigned) {
        return false;
    }
    if (avoid_goalie && robot->attribute.is_goalie) {
        return false;
    }
    return true;

}

void resetAssignment(WorldModel &world_model, bool preserve_goalie) {
    for (auto robot : world_model.friends.robots) {
        robot->attribute.is_assigned = false;
        if (preserve_goalie && robot->attribute.is_goalie) {
            robot->attribute.is_assigned = true;
        }
    }
}

std::shared_ptr<RobotNode> popBallClosest(WorldModel &world_model, bool avoid_goalie) {
    for (auto robot : world_model.ball_close_friends) {

        if (isValidRobot(robot, avoid_goalie)) {
            robot->attribute.is_assigned = true;
            return robot;
        }
    }
    return nullptr;
}

std::shared_ptr<RobotNode> popMaxPassRate(WorldModel &world_model, Point &receive_point, bool avoid_goalie) {
    for (auto p : world_model.field_analyzer.max_pass_rate) {
        auto robot = world_model.friends.robots[p.id];
        if (isValidRobot(robot, avoid_goalie)) {
            receive_point = p.pos;
            robot->attribute.is_assigned = true;
            return robot;
        }
    }
    return nullptr;
}

std::shared_ptr<RobotNode> popMaxTotalRate(WorldModel &world_model, Point &point, bool avoid_goalie){
    for (auto p : world_model.field_analyzer.max_total_rate) {
        auto robot = world_model.friends.robots[p.id];
        if (isValidRobot(robot, avoid_goalie)) {
            point = p.pos;
            robot->attribute.is_assigned = true;
            return robot;
        }
    }
    return nullptr;
}

std::shared_ptr<RobotNode> popGoalie(WorldModel &world_model) {
    auto goalie = world_model.friends.getGoalie();
    if (isValidRobot(goalie, false)) {
        return goalie;
    }
    return nullptr;
}

std::vector<std::shared_ptr<RobotNode>> popRestRobots(WorldModel &world_model, bool avoid_goalie) {
    std::vector<std::shared_ptr<RobotNode>> robots;
    for (auto robot : world_model.friends.robots) {
        if (!robot->is_disappeared && !robot->attribute.is_assigned) {
            if (avoid_goalie && robot->attribute.is_goalie) {
                continue;
            }
            robot->attribute.is_assigned = true;
            robots.push_back(robot);
        }
    }
    return robots;
}
