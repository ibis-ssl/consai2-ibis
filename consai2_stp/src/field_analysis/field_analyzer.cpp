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

#include <world_model/world_model.h>
#include <world_model/constants.h>
#include <field_analysis/field_analyzer.h>

FieldAnalyzer::FieldAnalyzer() :
        HEIGHT(static_cast<int>(Constants::field_length() / RESOLUTION)),
        WIDTH(static_cast<int>(Constants::field_width() / RESOLUTION)) {

    int size = HEIGHT * WIDTH;
    for (int i = 0; i < Constants::max_id(); i++) {
        for (int j = 0; j < 5; j++) {
            pass_map[j][i].reserve(size);
            for (int k = 0; k < size; k++) {
                pass_map[j][i].emplace_back(0.f);
            }
        }
        for (int j = 0; j < size; j++) {
            total_map[i].emplace_back(0.f);
            pass_total_map[i].emplace_back(0.f);
        }
    }

    for (int i = 0; i < 4; i++) {
        int size = HEIGHT * WIDTH;
        shoot_map[i].reserve(size);
        for (int j = 0; j < size; j++) {
            shoot_map[i].emplace_back(0.0f);
        }
    }

    for (int i = 0; i < size; i++) {
        shoot_total_map.emplace_back(0.f);
    }

    ros::NodeHandle nh;
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/consai2_stp/cost_map", 10);

    map4rviz.header.frame_id = "map";
    map4rviz.info.resolution = RESOLUTION;
    map4rviz.info.height = HEIGHT;
    map4rviz.info.width = WIDTH;
}


void FieldAnalyzer::update(WorldModel &world_model) {

    for (int i = 0; i < Constants::max_id(); i++) {
        if (world_model.friends.robots.at(i)->is_disappeared) {
            std::fill(total_map[i].begin(), total_map[i].end(), 0.f);
        } else {
            std::fill(total_map[i].begin(), total_map[i].end(), 1.f);
        }
    }
    updatePassCondition(world_model);
    updateShootCondition(world_model);
}

void FieldAnalyzer::updatePassCondition(WorldModel &world_model) {
    for (int robot_id = 0; robot_id < Constants::max_id(); robot_id++) {
        max_pass_rate[robot_id].value = 0.f;
        max_pass_rate[robot_id].id = robot_id;
        max_pass_rate[robot_id].pos << 0.f, 0.f;

        if (!world_model.friends.robots.at(robot_id)->is_disappeared) {
            Point target;
            float total_rate, rate;
            for (int h = 0; h < HEIGHT; h++) {
                for (int w = 0; w < WIDTH; w++) {
                    total_rate = 1.f;
                    target << h * RESOLUTION - Constants::half_field_length(),
                            w * RESOLUTION - Constants::half_field_width();
                    for (int map_id = 1; map_id <= 5; map_id++) {
                        rate = pass_rate.getSuccessRate(map_id, world_model, target,
                                                        world_model.friends.robots[robot_id]);
                        pass_map[map_id][robot_id].at(h * WIDTH + w) = rate;
                        total_rate *= rate;
                    }

                    pass_total_map[robot_id].at(h * WIDTH + w) = total_rate;
                    total_map[robot_id].at(h * WIDTH + w) *= total_rate;

                    if (total_rate > max_pass_rate[robot_id].value) {
                        max_pass_rate[robot_id].value = total_rate;
                        max_pass_rate[robot_id].pos = target;
                    }
                }
            }
        }
    }

    std::sort(max_pass_rate.begin(), max_pass_rate.end(), [](auto const &lhs, auto const &rhs) {
        return lhs.value > rhs.value;
    });
}

void FieldAnalyzer::updateShootCondition(WorldModel &world_model) {
    max_shoot_rate.value = 0.f;
    max_shoot_rate.pos << 0.f, 0.f;

    for(int i = 0 ; i<Constants::max_id();i++){
        max_total_rate[i].value = 0.f;
        max_total_rate[i].id = i;
    }


    Point target;
    float total_rate, rate;
    for (int h = 0; h < HEIGHT; h++) {
        for (int w = 0; w < WIDTH; w++) {
            total_rate = 1.f;
            target << h * RESOLUTION - Constants::half_field_length(),
                    w * RESOLUTION - Constants::half_field_width();
            for (int map_id = 1; map_id <= 4; map_id++) {
                rate = shoot_rate.getSuccessRate(map_id, world_model, target);
                shoot_map[map_id].at(h * WIDTH + w) = rate;
                total_rate *= rate;
            }
            shoot_total_map.at(h * WIDTH + w) = total_rate;
            for (int robot_id = 0; robot_id < Constants::max_id(); robot_id++) {
                total_map[robot_id].at(h * WIDTH + w) *= total_rate;
                total_rate = total_map[robot_id].at(h * WIDTH + w);
                if(max_total_rate[robot_id].value < total_rate){
                    max_total_rate[robot_id].value = total_rate;
                    max_total_rate[robot_id].pos << h * RESOLUTION - Constants::half_field_length(),
                                                    w * RESOLUTION - Constants::half_field_width();
                }
            }

            if (total_rate > max_shoot_rate.value) {
                max_shoot_rate.value = total_rate;
                max_shoot_rate.pos = target;
            }
        }
    }

    std::sort(max_total_rate.begin(), max_total_rate.end(), [](auto const &lhs, auto const &rhs) {
        return lhs.value > rhs.value;
    });
}





