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
// Created by hans on 2019/09/01.
//

#pragma once

//-------------include----------------//
#include <world_model/pose2d.h>
#include <world_model/world_model.h>


class TargetPointBase {
public:
    virtual Point getPoint(const WorldModel &world_model) = 0;
};

class TargetSegmentBase {
public:
    virtual Segment getSegment(const WorldModel &world_model) = 0;
};

class TargetBall : public TargetPointBase {
public:
    virtual Point getPoint(const WorldModel &world_model) override;
};

class TargetBallLine : public TargetSegmentBase {
public:
    virtual Segment getSegment(const WorldModel &world_model) override;
};

class TargetFriendRobot : public TargetPointBase {
public:
    uint8_t id;
public:
    TargetFriendRobot(uint8_t id) : id(id) {}

    virtual Point getPoint(const WorldModel &world_model) override;
};

class TargetEnemyRobot : public TargetPointBase {
public:
    uint8_t id;
public:
    TargetEnemyRobot(uint8_t id) : id(id) {}

    virtual Point getPoint(const WorldModel &world_model) override;
};

class TargetPoint : public TargetPointBase {
public:
    Point point;
public:
    explicit TargetPoint(Point point) : point(point) {}

    virtual Point getPoint(const WorldModel &world_model) override {
        return point;
    }
};

class TargetModule {
private:
    std::shared_ptr<TargetPointBase> base;
public:
    TargetModule() {}

    explicit TargetModule(std::shared_ptr<TargetPointBase> base) : base(base) {}

    Point getPoint(const WorldModel &world_model) {
        return base->getPoint(world_model);
    }

    static TargetModule buildBall() {
        auto ball = std::make_shared<TargetBall>();
        auto module = TargetModule(ball);
        return module;
    }

    static TargetModule buildFriend(uint8_t id) {
        auto friend_robot = std::make_shared<TargetFriendRobot>(id);
        auto module = TargetModule(friend_robot);
        return module;
    }

    static TargetModule buildEnemy(uint8_t id) {
        auto enemy_robot = std::make_shared<TargetEnemyRobot>(id);
        auto module = TargetModule(enemy_robot);
        return module;
    }

    static TargetModule buildPoint(Point point) {
        auto module = TargetModule(std::make_shared<TargetPoint>(point));
        return module;
    }
};

class TargetLineModule {
private:
    std::shared_ptr<TargetSegmentBase> base;
public:
    TargetLineModule() {}

    explicit TargetLineModule(std::shared_ptr<TargetSegmentBase> base) : base(base) {}

    Segment getLine(const WorldModel &world_model) {
        return base->getSegment(world_model);
    }

    static TargetLineModule buildBallLine() {
        auto ball_line = std::make_shared<TargetBallLine>();
        auto module = TargetLineModule(ball_line);
        return module;
    }
};
