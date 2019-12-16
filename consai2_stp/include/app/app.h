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
// Created by hans on 2019/11/30.
//

#pragma once

//-------------include----------------//
#include <ros/ros.h>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include <play_executer.h>
#include <imgui_impl_glfw.h>
//------------namespace---------------//
//--------------class-----------------//


class ImGuiApp {
public:
    GLFWwindow *window;
    const char *glsl_version;
    ros::Rate *rate;
    PlayExecuter *play_executer;
    ImVec4 clear_color;
public:
    ImGuiApp() {}

    int initilizeGL();

    void initilizeImGui();

    void initilizeROS(int argc, char **argv,Play *reset_play);

    void spin();

    void finalize();

    void visualizeField(const WorldModel &world_model, ImDrawList *draw_list);
};
