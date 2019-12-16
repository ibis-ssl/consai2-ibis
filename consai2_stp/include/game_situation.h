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
// Created by hans on 2019/09/11.
//

#pragma once

enum class GameSituation {
    HALT = 0,
    STOP = 1,
    FORCE_START = 3,
    OUR_KICKOFF_PREPARATION = 11,
    OUR_KICKOFF_START = 12,
    OUR_PENALTY_PREPARATION = 13,
    OUR_PENALTY_START = 14,
    OUR_DIRECT_FREE = 15,
    OUR_INDIRECT_FREE = 16,
    OUR_TIMEOUT = 17,
    OUR_GOAL = 18,
    OUR_BALL_PLACEMENT = 19,

    THEIR_KICKOFF_PREPARATION = 21,
    THEIR_KICKOFF_START = 22,
    THEIR_PENALTY_PREPARATION = 23,
    THEIR_PENALTY_START = 24,
    THEIR_DIRECT_FREE = 25,
    THEIR_INDIRECT_FREE = 26,
    THEIR_TIMEOUT = 27,
    THEIR_GOAL = 28,
    THEIR_BALL_PLACEMENT = 29,
};

enum class InplaySituation{
    NORMAL,
    BALL_IN_OUR_DEFENSE,
    BALL_IN_THEIR_DEFENSE,
};
