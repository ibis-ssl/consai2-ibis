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
// Created by hans on 2019/08/17.
//

#pragma once

//-------------include----------------//
#include <world_model/constants.h>

//------------namespace---------------//
//--------------class-----------------//

enum class BallState {
    NONE,
    HOLD,
    SHOOTING,
};

class BallInfo {
public:
    struct {
        Point pos;
        Velocity vel;
    } pose;
    struct {
        float hysteresis_meter = 0.05f;
        float moved_threshold_meter = 0.03f;
        float moving_speed_threshold_mps = 1.0f;
        float moving_speed_hysteresis_mps = 0.3f;
    } params;

    Point ball_initial_pos;

    BallState state;

    bool is_in_field;
    bool is_moved;//Placementされてから動いたかどうか
    bool is_moving;
//    bool has_lost = false;
//    bool is_in_our_defence;
//    bool is_in_their_defence;
    bool is_ours;//ボールの所有権を持っているかどうか

public:
    bool isBallInField() {
        static float abs_x, abs_y;
        abs_x = fabs(pose.pos.x());
        abs_y = fabs(pose.pos.y());

        if (is_in_field) {
            if ((abs_x > Constants::half_field_length() + params.hysteresis_meter) ||
                (abs_y > Constants::half_field_width() + params.hysteresis_meter)) {
                is_in_field = false;
            }
        } else {
            if ((abs_x < Constants::half_field_length() - params.hysteresis_meter) &&
                (abs_y < Constants::half_field_width() - params.hysteresis_meter)) {
                is_in_field = true;
            }
        }
        return is_in_field;
    }

    void setBallInitialPos(const Point &pos) {
        ball_initial_pos = pos;
        is_moved = false;
    }

    bool isBallMoved() {
        Point pos;
        pos << pose.pos.x(), pose.pos.y();
        float delta = (ball_initial_pos - pos).norm();
        if (delta > params.moved_threshold_meter) {
            is_moved = true;
        }
        return is_moved;
    }

    bool isBallMoving() {
        float ball_speed = pose.vel.norm();

        if (!is_moving && ball_speed > params.moving_speed_threshold_mps + params.moving_speed_hysteresis_mps) {
            is_moving = true;
        } else if (is_moving && ball_speed < params.moving_speed_threshold_mps - params.moving_speed_threshold_mps) {
            is_moving = false;
        }
        return is_moving;
    }
};
