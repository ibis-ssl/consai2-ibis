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
// Created by hans on 2019/10/24.
//
#include <play_executer.h>


void PlayExecuter::update() {
    world_model.update();
    if (world_model.isAvailable()) {
        execute();
        if (play->hasFinished(world_model) || world_model.has_play_finished) {
            selectPlay();
        }
        send();
    }

}


void PlayExecuter::selectPlay() {
#if 1
    //test
    ROS_INFO("PLAY SWITCHED!");

//    play.reset(new ResetPlay());
//    play->initialize(world_model, world_model.friends);
#else
    auto ref = world_model.getDecodedReferee();
        if(ref.referee_id != static_cast<uint8_t>(situation)){
            ROS_INFO("REFEREE : %s",ref.referee_text.c_str());
            for (auto robot : world_model.friends.robots) {
                robot->behavior.clearChild();
            }
            play = play_book.getPlay(static_cast<GameSituation>(ref.referee_id));
            situation = static_cast<GameSituation>(ref.referee_id);
        }
#endif
    world_model.has_play_finished = false;
}


void PlayExecuter::execute() {
    for (auto robot : world_model.friends.robots) {
        if (robot->behavior.status == Status::RUNNING)
            robot->behavior.status = robot->behavior.run(world_model, robot->attribute.id);
    }
}


void PlayExecuter::send() {
    world_model.publishControlTargets();
    world_model.publishBehaviorTree();
}
