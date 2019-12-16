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
// Created by hans on 2019/11/23.
//

#pragma once

//-------------include----------------//
//------------namespace---------------//
//--------------class-----------------//

#include <vector>
#include <algorithm>

class Interval {
private:
    std::vector<float> uppers;
    std::vector<float> lowers;

public:
    Interval() {}

    ~Interval() {}

    void append(float lower, float upper) {
        uppers.emplace_back(upper);
        lowers.emplace_back(lower);

        std::sort(uppers.begin(), uppers.end());
        std::sort(lowers.begin(), lowers.end());
        for (size_t i = 1; i < uppers.size(); i++) {
            //重なっている
            if (uppers[i - 1] > lowers[i]) {
                uppers[i - 1] = uppers[i];
                lowers.erase(lowers.begin() + i);
                uppers.erase(uppers.begin() + i);
                i--;
            }
        }
    }

    void erase(float lower, float upper) {
        for (size_t i = 0; i < uppers.size(); i++) {
            //完全消去
            if (uppers[i] < upper && lowers[i] > lower) {
                lowers.erase(lowers.begin() + i);
                uppers.erase(uppers.begin() + i);
                i--;
                continue;
            }
            //中抜き
            if (uppers[i] > upper && lowers[i] < lower) {
                uppers.emplace_back(lower);
                lowers.emplace_back(upper);
                std::sort(uppers.begin(), uppers.end());
                std::sort(lowers.begin(), lowers.end());
            }

            //上限修正
            if (lower < uppers[i] && upper > uppers[i]) {
                uppers[i] = lower;
            }
            //下限修正
            if (lowers[i] < upper && lowers[i] > lower) {
                lowers[i] = upper;
            }
        }
        std::sort(uppers.begin(), uppers.end());
        std::sort(lowers.begin(), lowers.end());
    }

    float getWidth() {
        float width = 0.f;
        for (size_t i = 0; i < lowers.size(); i++) {
            width += uppers[i] - lowers[i];
        }
        return width;
    }
};
