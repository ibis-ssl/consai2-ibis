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
#include <string>
#include <vector>
#include <game_situation.h>
//------------namespace---------------//
//--------------class-----------------//

class WorldModel;

class TeamInfo;


/**
 * @note
 */
class Play {
public:
    std::string name;
public :
    explicit Play(std::string name) : name(name) {}

    virtual bool initialize(WorldModel &world_model, TeamInfo &friends) = 0;

    virtual bool hasFinished(const WorldModel &world_model) = 0;
};

class RefereePlay : public Play{
public:
    explicit RefereePlay(std::string name):Play(name){}
    virtual std::vector<GameSituation> getTriggers() const = 0;
};

class InplayPlay : public Play{
public:
    explicit InplayPlay(std::string name):Play(name){}
    virtual std::vector<InplaySituation> getTriggers() const = 0;
};
