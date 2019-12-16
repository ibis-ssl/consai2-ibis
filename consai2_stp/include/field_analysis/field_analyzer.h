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
// Created by hans on 2019/10/22.
//

#pragma once

//-------------include----------------//
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <field_analysis/CMD14TDP_3/conditions.h>

struct PosValueID {
    Point pos;
    float value;
    uint8_t id;

};

class FieldAnalyzer {
public:
    nav_msgs::OccupancyGrid map4rviz;
    ros::Publisher map_publisher;
    const float RESOLUTION = 0.2f;
    const float HEIGHT;
    const float WIDTH;
    std::vector<float> pass_map[5][Constants::max_id()];
    std::vector<float> pass_total_map[Constants::max_id()];
    std::array<std::vector<float>,Constants::max_id()> total_map;
    std::vector<float> shoot_map[4];
    std::vector<float> shoot_total_map;
    condition::PassConditions pass_rate;
    condition::ShootConditions shoot_rate;

public:
    std::array<PosValueID, Constants::max_id()> max_pass_rate;
    std::array<PosValueID, Constants::max_id()> max_total_rate;
    PosValueID max_shoot_rate;

public:
    FieldAnalyzer();

    void update(WorldModel &world_model);

    void updatePassCondition(WorldModel &world_model);

    void updateShootCondition(WorldModel &world_model);

};
