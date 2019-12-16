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
#include <world_model/ball_info.h>
#include <world_model/world_model.h>
#include <boost/numeric/interval.hpp>
#include <field_analysis/CMD14TDP_3/conditions.h>
#include <utils/interval.h>
#include <world_model/tool.h>

namespace condition {

    float ShootConditions::getSuccessRate(const WorldModel &world_model, Point target) {
        PassRestriction restriction;
        restriction.maximum_vel = 8.0f;
        restriction.minimum_vel = 0.0f;
        float total_rate = 1.0f;
        float rate;

        std::clamp(rate = getC1SuccessRate(world_model, target, restriction), 0.f, 1.f);
        total_rate *= rate;

        std::clamp(rate = getC2SuccessRate(world_model, target, restriction), 0.f, 1.f);
        total_rate *= rate;

//        std::clamp(rate = getC3SuccessRate(world_model, target, restriction),0.f,1.f);
//        total_rate *= rate;
//
        std::clamp(rate = getC4SuccessRate(world_model, target, restriction), 0.f, 1.f);
        total_rate *= rate;

        return total_rate;
    }

    float ShootConditions::getSuccessRate(int map_id, const WorldModel &world_model, Point target) {
        PassRestriction restriction;
        restriction.maximum_vel = 8.0f;
        restriction.minimum_vel = 0.0f;
        float rate;
        switch (map_id) {
            case 1:
                rate = getC1SuccessRate(world_model, target, restriction);
                break;

            case 2:
                rate = getC2SuccessRate(world_model, target, restriction);
                break;
            case 3:
                rate = getC3SuccessRate(world_model, target, restriction);
                break;

            case 4:
                rate = getC4SuccessRate(world_model, target, restriction);
                break;
            default:
                return 0.f;
        }
        std::clamp(rate, 0.f, 1.f);
        return rate;
    }

    /*
     * xからのシュートが敵キーパーのブロックよりも早い
     */
    float ShootConditions::getC1SuccessRate(const WorldModel &world_model, Point target, PassRestriction &restriction) {
        //TODO : 書く
        auto goalie = world_model.enemys.getGoalie();
        float goalie_angle = tool::getAngle(goalie->pose.pos - target);
        float angle_1 = tool::getAngle(Constants::Their::Goal::lower() - target);
        float angle_2 = tool::getAngle(Constants::Their::Goal::upper() - target);
        float diff_angle = std::max(std::abs(angle_1 - goalie_angle), std::abs(angle_2 - goalie_angle));

        float dist = bg::distance(target, Constants::Their::Goal::center());
        return 15.f * diff_angle / (dist + 8.0f);
    }

    /*
     * xからゴールを見た時に十分な角度を占めている
     */
    float ShootConditions::getC2SuccessRate(const WorldModel &world_model, Point target, PassRestriction &restriction) {
        Interval goal_range;

        float lower_angle = tool::getAngle(Constants::Their::Goal::lower() - target);
        float upper_angle = tool::getAngle(Constants::Their::Goal::upper() - target);

        goal_range.append(lower_angle, upper_angle);

        for (auto robot : world_model.enemys.robots) {
            if (robot->is_disappeared) {
                continue;
            }
            //targetから見てゴール側にいる敵ロボットのみ．それ以外はスキップ
            if ((robot->pose.pos - target).dot((Constants::Their::Goal::center() - target).normalized()) < 0.f) {
                continue;
            }

            float d = bg::distance(robot->pose.pos, target);
            float r = (Constants::robot_radius() * d) /
                      std::sqrt(d * d - Constants::robot_radius() * Constants::robot_radius());
            Point v_vec = tool::getVerticalVec((robot->pose.pos - target).normalized());
            float angle_1 = tool::getAngle(robot->pose.pos - target - (v_vec * r));
            float angle_2 = tool::getAngle(robot->pose.pos - target + (v_vec * r));
            goal_range.erase(std::min(angle_1, angle_2), std::max(angle_1, angle_2));
        }

        float goal_angle = goal_range.getWidth();
        //27度あったらいい方
        return goal_angle / (M_PI * 0.15f);
    }

    /*
     * 敵にシュートをブロックされない
     */
    float ShootConditions::getC3SuccessRate(const WorldModel &world_model, Point target, PassRestriction &restriction) {
        //TODO : 書く
        return 1.f;
    }

    /*
     * 敵にボールを奪われない
     */
    float ShootConditions::getC4SuccessRate(const WorldModel &world_model, Point target, PassRestriction &restriction) {
        float min_dist = 1000.f, dist;
        for (auto robot : world_model.enemys.robots) {
            dist = bg::distance(target, robot->pose.pos);
            min_dist = std::min(min_dist, dist);
        }
        return min_dist / 2.f;
    }
}
