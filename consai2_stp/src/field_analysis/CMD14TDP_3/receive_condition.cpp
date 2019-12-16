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
#include <world_model/tool.h>

#include <field_analysis/CMD14TDP_3/conditions.h>

namespace condition {
    float PassConditions::getSuccessRate(const WorldModel &world_model, Point target,
                                         std::shared_ptr<RobotNode> R) {
        PassRestriction restriction;
        restriction.maximum_vel = 8.0f;
        restriction.minimum_vel = 0.0f;
        float total_rate = 1.0f;
        float rate;

//        std::clamp(rate = getC1SuccessRate(world_model, target, R, restriction),0.f,1.f);
//        total_rate *= rate;

        std::clamp(rate = getC2SuccessRate(world_model, target, R, restriction), 0.f, 1.f);
        total_rate *= rate;

        std::clamp(rate = getC3SuccessRate(world_model, target, R, restriction), 0.f, 1.f);
        total_rate *= rate;

        std::clamp(rate = getC4SuccessRate(world_model, target, R, restriction), 0.f, 1.f);
        total_rate *= rate;

        std::clamp(rate = getC5SuccessRate(world_model, target, R, restriction), 0.f, 1.f);
        total_rate *= rate;

        return total_rate;
    }

    float PassConditions::getSuccessRate(int map_id, const WorldModel &world_model, Point target,
                                         std::shared_ptr<RobotNode> R) {
        PassRestriction restriction;
        restriction.maximum_vel = 8.0f;
        restriction.minimum_vel = 0.0f;
        float rate;
        switch (map_id) {
            case 1:
                rate = getC1SuccessRate(world_model, target, R, restriction);
                break;

            case 2:
                rate = getC2SuccessRate(world_model, target, R, restriction);
                break;
            case 3:
                rate = getC3SuccessRate(world_model, target, R, restriction);
                break;

            case 4:
                rate = getC4SuccessRate(world_model, target, R, restriction);
                break;
            case 5:
                rate = getC5SuccessRate(world_model, target, R, restriction);
                break;
            default:
                return 0.f;
        }
        std::clamp(rate, 0.f, 1.f);
        return rate;
    }

    /**
     * Rがxにどの敵よりも早くたどり着く確率
     */
    float PassConditions::getC1SuccessRate(const WorldModel &world_model, Point target,
                                           std::shared_ptr<RobotNode> R, PassRestriction &restriction) {
        Point R_target_direction = tool::getDirectonNorm(R->pose.pos, target);
        float R_v0 = R->vel.pos.dot(R_target_direction);
        float R_reach_t = tool::getReachTime(bg::distance(R->pose.pos, target), R_v0, R->specs.max_acc);
        float enemy_reach_t = 100.f;

        for (auto enemy : world_model.enemys.robots) {
            Point target_direction = tool::getDirectonNorm(enemy->pose.pos, target);
            float v0 = enemy->vel.pos.dot(target_direction);
            float reach_t = tool::getReachTime(bg::distance(enemy->pose.pos, target), v0, enemy->specs.max_acc);
            if (reach_t < enemy_reach_t) {
                enemy_reach_t = reach_t;
            }
        }
        constexpr float THRESHOLD_TIME = 1.f;
        if (R_reach_t > enemy_reach_t) {
            return 1.0f - (R_reach_t - enemy_reach_t) / THRESHOLD_TIME;
        }
        return 1.f;
    }

    /**
     * パスが敵にカットされない
     * 確率的には0か1かしか返さず，パスの制約条件を設定することによって成功確率を表現する
     *
     * P-x間の線分上に敵がボールより早く到着しない
     * TODO : チップキックで宙に浮くときはロボットの飛越も考慮にいれること
     */
    float PassConditions::getC2SuccessRate(const WorldModel &world_model, Point target,
                                           std::shared_ptr<RobotNode> R, PassRestriction &restriction) {
        auto ball_pos = world_model.ball.pose.pos;
        Segment ball_line = tool::getSegment(target, ball_pos);

        //敵ロボットそれぞれについて評価
        float min_dist = 100.0f;
        float min_reach_t = 100.f;
        float max_min_vel = 0.0f;
        for (auto enemy: world_model.enemys.robots) {
            //存在していないロボットは除去
            if (enemy->is_disappeared) {
                continue;
            }

            //enemyがボールの進行方向より後ろに居た場合,評価対象から除外
            if ((target - ball_pos).dot(enemy->pose.pos - ball_pos) < 0) {
                continue;
            }

            //enemyからball lineへの最短到達点(=垂線の足となる点)を求める
            ClosestPoint result;
            bg::closest_point(enemy->pose.pos, ball_line, result);

            float distance_from_ball_to_closest = bg::distance(ball_pos, result.closest_point);
//        float distance_from_enemy_to_closest = bg::distance(enemy->pose.pos,result.closest_point);
            float distance_from_enemy_to_closest = result.distance;
//            float distance_from_enemy_to_ball = bg::distance(ball_pos, enemy->pose.pos);
            if (min_dist > distance_from_enemy_to_closest) {
                min_dist = distance_from_enemy_to_closest;
            }

            //enemyの速度のうち，ball_lineに向かう速度成分を求める
            //enemyがボールをカットするのに向かうべき単位方向ベクトルを求める(enemyの位置からball_lineの下ろした垂線の方向)
            Point best_direction = tool::getVerticalVec((target - ball_pos).normalized());
            //方向判定(getVerticalVecでは垂直にする方向を選べないため，向きに応じて処理を分ける必要がある)
            float dot = best_direction.dot(enemy->pose.pos - ball_pos);
            //内積が負のとき，best_directionがenemy->ball_lineの向きになっている
            if (dot > 0) {
                best_direction = -best_direction;
            }

            //ball_lineに向かう初速度成分
            float v0 = enemy->vel.pos.dot(best_direction);
            //enemyのball_lineへの最短到達時間
            float enemy_reach_t = tool::getReachTime(distance_from_enemy_to_closest, v0, enemy->specs.max_acc);
            if (min_reach_t > enemy_reach_t) {
                min_reach_t = enemy_reach_t;
            }

            //ballがとられないようにするためのballの最低速度
            float min_ball_vel = distance_from_ball_to_closest / enemy_reach_t;
            if (max_min_vel < min_ball_vel) {
                max_min_vel = min_ball_vel;
            }
            //最大速度条件に抵触する場合
            if (restriction.maximum_vel < min_ball_vel) {
                return 0;
            }

            //最低速度条件を更新
            if (restriction.minimum_vel < min_ball_vel) {
                restriction.minimum_vel = min_ball_vel;
            }
        }

//    //条件が矛盾している場合
        if (restriction.minimum_vel > restriction.maximum_vel) {
            return 0;
        }

        return (4.f - restriction.minimum_vel) / 4.f;
    }

    /**
     * パスがRが反応して受け取るために十分に長い
     * パスのmax_velの制約条件を設定
     *
     * ボールがxに移動する時間 > Rがxに移動する時間
     */
    float PassConditions::getC3SuccessRate(const WorldModel &world_model, Point target,
                                           std::shared_ptr<RobotNode> R, PassRestriction &restriction) {

        Point target_direction = tool::getDirectonNorm(R->pose.pos, target);
        float v0 = R->vel.pos.dot(target_direction);
        //最短到着時間
        float reach_t = tool::getReachTime(bg::distance(target, R->pose.pos), v0, R->specs.max_acc, R->specs.max_vel);

        if (reach_t == 0.f) return 0.f;
        //ボールが出しても良い最大速度(等速直線運動を仮定)
        // (この速度を超えてパスをすると受け取りロボットがボールに追いつかない)
        Point ball_pos = world_model.ball.pose.pos;
        float max_ball_vel = bg::distance(ball_pos, target) / reach_t;

        //条件更新(速度の上限)
        if (max_ball_vel < restriction.maximum_vel) {
            restriction.maximum_vel = max_ball_vel;
        }

        //条件が矛盾している場合
        if (restriction.minimum_vel > restriction.maximum_vel) {
            return 0.f;
        }

        return restriction.maximum_vel / 2.0f;
    }

    /**
     * パスが十分に正確に実行できる長さより短い
     *
     * 予め決められたしきい値 d_maxより短いこと
     * TODO : 外部からd_maxが設定できるようにする
     */
    float PassConditions::getC4SuccessRate(const WorldModel &world_model, Point target,
                                           std::shared_ptr<RobotNode> R, PassRestriction &restriction) {
        constexpr float d_max = 10.0f;
        Point ball = world_model.ball.pose.pos;

        float d = (ball - target).norm();

        if (d > d_max) {
            return 0.0f;
        }

        //TODO : 距離を確率に反映させる
        return 1.0f;
    }

    /**
     * taregtがパスを受け取るのに適した場所であるか
     *
     * ❌　: defence area　に近い
     * ❌　: lineに近い
     * ❌　: 味方に近すぎる
     */
    float PassConditions::getC5SuccessRate(const WorldModel &world_model, Point target,
                                           std::shared_ptr<RobotNode> R, PassRestriction &restriction) {

        //ペナルティエリア判定()
#if 0
        auto penalty_area = Constants::Their::Penalty::getArea();
        //TODO : 距離は適当なのでパラメータ化
        constexpr float PENALTY_THRESHOLD = 0.3f;
        if (bg::distance(target, penalty_area) < PENALTY_THRESHOLD) {
            return 0;
        }
#endif
        //Lineの接近判定
        //TODO : 距離は適当なのでパラメータ化
        constexpr float LINE_THRESHOLD = 0.3f;
        if ((Constants::half_field_length() - abs(target.x())) < LINE_THRESHOLD) {
            return 0.f;
        }
        if ((Constants::half_field_width() - abs(target.y())) < LINE_THRESHOLD) {
            return 0.f;
        }

        if(tool::isInTheirDefenseArea(target) || tool::isInOurDefenseArea(target)){
            return 0.f;
        }

        //TODO : 味方判定 (なくてもいけそうな気はする)

        return 1.f;
    }
}

