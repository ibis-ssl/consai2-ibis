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
// Created by hans on 2019/09/06.
//

#pragma once

//-------------include----------------//
#include <behavior_tree/status_converter/status_converter.h>

//------------namespace---------------//
//--------------class-----------------//
class Inverter : StatusConverter {
public:
    explicit Inverter(std::shared_ptr<Component> base) : StatusConverter("Inverter", base) {}

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        base->status = base->run(world_model, my_id);
        if (base->status == Status::SUCCESS) {
            this->status = Status::FAILURE;
        } else if (base->status == Status::FAILURE) {
            this->status = Status::SUCCESS;
        } else {
            this->status = Status::RUNNING;
        }
        return status;
    }
};
