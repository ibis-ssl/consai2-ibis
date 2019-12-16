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
// Created by hans on 2019/09/03.
//

#pragma once

//-------------include----------------//
#include <behavior_tree/parallel_all.h>
#include <utils/eigen_adapter.h>
#include <ros/ros.h>


//------------namespace---------------//
//--------------class-----------------//
class TacticPassReceive : public ParallelAll {
protected:
    const float CORRECTION_ENABLE_DISTANCE;
    Point receive_point;
public:
    TacticPassReceive(Point receive_point, float correction_enable_distance = 0.0f);

    static std::shared_ptr<TacticPassReceive> build(Point receive_point, float correction_enable_distance = 0.0f) {
        ROS_WARN("PASS RECEIVE : %f\t%f", receive_point.x(), receive_point.y());
        return std::make_shared<TacticPassReceive>(receive_point, correction_enable_distance);
    }
};
