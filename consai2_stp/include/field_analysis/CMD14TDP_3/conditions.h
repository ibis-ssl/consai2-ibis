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
// Created by hans on 2019/08/19.
//

#pragma once

//pre-declaration
class WorldModel;

class RobotNode;

/**
 * CMD 2014 TDP Section 3
 */
namespace condition {
    struct PassRestriction {
        float minimum_vel = 0.0f;
        float maximum_vel = 8.0f;
    };

    class PassConditions {
    public:
        float getSuccessRate(const WorldModel &world_model, Point target, std::shared_ptr<RobotNode> R);

        float getSuccessRate(int map_id,const WorldModel &world_model, Point target, std::shared_ptr<RobotNode> R);

    private:
        float getC1SuccessRate(const WorldModel &world_model, Point target, std::shared_ptr<RobotNode> R,
                                   PassRestriction &restriction);

        float getC2SuccessRate(const WorldModel &world_model, Point target, std::shared_ptr<RobotNode> R,
                                   PassRestriction &restriction);

        float getC3SuccessRate(const WorldModel &world_model, Point target, std::shared_ptr<RobotNode> R,
                                   PassRestriction &restriction);

        float getC4SuccessRate(const WorldModel &world_model, Point target, std::shared_ptr<RobotNode> R,
                                   PassRestriction &restriction);

        float getC5SuccessRate(const WorldModel &world_model, Point target, std::shared_ptr<RobotNode> R,
                                   PassRestriction &restriction);
    };

    class ShootConditions {
    public:
        float getSuccessRate(const WorldModel &world_model, Point target);

        float getSuccessRate(int map_id,const WorldModel &world_model, Point target);

    private:
        float getC1SuccessRate(const WorldModel &world_model, Point target,PassRestriction &restriction);

        float getC2SuccessRate(const WorldModel &world_model, Point target,PassRestriction &restriction);

        float getC3SuccessRate(const WorldModel &world_model, Point target,PassRestriction &restriction);

        float getC4SuccessRate(const WorldModel &world_model, Point target,PassRestriction &restriction);
    };

}
