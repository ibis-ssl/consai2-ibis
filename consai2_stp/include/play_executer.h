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
// Created by hans on 2019/09/06.
//

#pragma once

//-------------include----------------//
#include <play/play_book.h>
#include <thread>

//------------namespace---------------//
//--------------class-----------------//

class PlayExecuter {
public:
    WorldModel world_model;
    std::unique_ptr<Play> play;
//    bool should_select_play = true;
    PlayBook play_book;
    GameSituation ref_situation = GameSituation::HALT;
    InplaySituation inplay_situation = InplaySituation::NORMAL;
public:
    PlayExecuter(Play *reset_play) {

        int cnt = 0;
        while (!world_model.has_robot_data_received && cnt < 5) {
            ros::spinOnce();
            update();
            if (world_model.has_robot_data_received) {
                cnt++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        play.reset(reset_play);
        play->initialize(world_model, world_model.friends);

    }

    void update();

    void selectPlay();

    void execute();

    void send();
};
