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
// Created by hans on 2019/08/16.
//

#pragma once

//-------------include----------------//
#include <composite/composite.h>

//------------namespace---------------//
//--------------class-----------------//

/**
 * 逐次実行．（先頭のものが成功するまで次にいかない）
 * 全て成功すれば成功判定
 * 途中で失敗が出れば失敗判定
 */
class Sequence : public Composite {
public:
    Sequence() {
        name = "Sequence";
    }

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        for (auto &c : children) {
            if (c->status == Status::SUCCESS)
                continue;

            c->status = c->run(world_model, my_id);

            if (c->status != Status::SUCCESS) {
                if (c->status == Status::FAILURE) {
                    return Status::FAILURE;
                }
                return c->status;
            }
        }
        return Status::SUCCESS;
    }
};
