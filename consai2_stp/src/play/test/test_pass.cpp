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
#include <play/test/test_pass.h>
#include <tactic/pass_receive.h>
#include <tactic/kick_to_target.h>
#include <world_model/ball_info.h>
#include <world_model/assignment.h>

bool TestPass::initialize(WorldModel &world_model, TeamInfo &friends) {

    resetAssignment(world_model);

//    std::shared_ptr<RobotNode> pass_robot, receive_robot;
    auto pass_robot = popBallClosest(world_model, false);
    auto receive_robot = popMaxTotalRate(world_model, this->receive_point, false);



    this->receiver_id = receive_robot->attribute.id;

    receive_robot = friends.robots[receiver_id];
    pass_robot = world_model.ball_close_friends[0];

    ROS_INFO("TEST_PASS\tR : %d\tP : %d\t POS : %f\t%f", receive_robot->attribute.id, pass_robot->attribute.id,
             receive_point.x(), receive_point.y());

    //Pass Robot

    pass_robot->behavior.clearChild();
    auto child1 = TacticKickToTarget::build(TargetModule::buildPoint(receive_point), 0.7f);
    pass_robot->behavior.addChild(child1);
    //Receive Robot
    receive_robot->behavior.clearChild();
    receive_robot->behavior.status = Status::RUNNING;
//    ROS_INFO("%f\t%f",receive_point.x(),receive_point.y());
    auto child2 = TacticPassReceive::build(receive_point, 0.1f);
    receive_robot->behavior.addChild(child2);
    ROS_WARN("name : %s", receive_robot->behavior.children.front()->name.c_str());
    return true;
}

bool TestPass::hasFinished(const WorldModel &world_model) {
    auto receiver = world_model.friends.getRobot(receiver_id);
    auto pos = receiver->pose.pos;
    auto ball = world_model.ball.pose.pos;
    if (bg::distance(pos, ball) < 0.15) {
        return true;
    }
    return false;

}

