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
#include <glad/glad.h>
#include <app/app.h>
#include <strategy.h>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>


static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}


int ImGuiApp::initilizeGL() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;
    glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(1080, 720, "ibis AI", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    // Initialize OpenGL loader
    bool err = gladLoadGL() == 0;

    if (err) {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }
    return 0;
}


void ImGuiApp::initilizeImGui() {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void) io;
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
}


void ImGuiApp::initilizeROS(int argc, char **argv,Play *reset_play) {
    const std::string node_name = "consai2_stp";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    rate = new ros::Rate(30);

    ROS_INFO("START!");
    ROS_INFO("CONNECTING TO CONSAI2 & GRSIM");
    play_executer = new PlayExecuter(reset_play);
    play_executer->world_model.friends.goalie_id = 1;
    play_executer->world_model.enemys.goalie_id = 1;
    play_executer->world_model.friends.color = Color::BLUE;
    play_executer->world_model.enemys.color = Color::YELLOW;
    ROS_INFO("CONNECTED!");
}


void ImGuiApp::spin() {
    while (ros::ok() && !glfwWindowShouldClose(window)) {
        ros::spinOnce();
        play_executer->update();

        glfwPollEvents();
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {
            ImGui::Begin("Field View");
            ImGui::Text("%f", ImGui::GetIO().Framerate);
            visualizeField(play_executer->world_model, ImGui::GetWindowDrawList());
            ImGui::End();
        }

        play_executer->world_model.resetControlTargetBuilder();
        // Rendering
        ImGui::Render();

        int display_w, display_h;
        glfwMakeContextCurrent(window);
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwMakeContextCurrent(window);
        glfwSwapBuffers(window);
        rate->sleep();
    }
}


void ImGuiApp::finalize() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}


void ImGuiApp::visualizeField(const WorldModel &world_model, ImDrawList *draw_list) {

    float scale = ImGui::GetWindowContentRegionWidth() / 13.f;
    Point offset;
    offset << ImGui::GetItemRectMin().x + 6.5f * scale, ImGui::GetItemRectMin().y + 5.0 * scale;

    for (int h = 0; h < world_model.field_analyzer.HEIGHT; h++) {
        for (int w = 0; w < world_model.field_analyzer.WIDTH; w++) {
            float value = 0;
            float tmp_value;
//            value = world_model.field_analyzer.shoot_total_map.at(h * world_model.field_analyzer.WIDTH + w);
            for (int i = 0; i < Constants::max_id(); i++) {
                tmp_value = world_model.field_analyzer.total_map[i].at(h * world_model.field_analyzer.WIDTH + w);
                if (value < tmp_value) {
                    value = tmp_value;
                }
            }

            Point pos1, pos2;
            pos1 << (h - 0.5f) * world_model.field_analyzer.RESOLUTION - Constants::half_field_length(),
                    (w - 0.5f) * world_model.field_analyzer.RESOLUTION - Constants::half_field_width();
            pos2 << pos1.x() + world_model.field_analyzer.RESOLUTION, pos1.y() + world_model.field_analyzer.RESOLUTION;
            pos1 *= scale;
            pos2 *= scale;
            pos1 += offset;
            pos2 += offset;
            draw_list->AddRectFilled({pos1.x(), pos1.y()}, {pos2.x(), pos2.y()},
                                     ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 0.0f, value)));
        }
    }

    ImU32 friend_color = ImGui::GetColorU32(ImVec4(0.5f, 0.5f, 1.0f, 1.0f));
    ImU32 enemy_color = ImGui::GetColorU32(ImVec4(1.0f, 0.5f, 0.5f, 1.0f));

    for (auto &&robot : world_model.friends.robots | boost::adaptors::indexed()) {
        if (!robot.value()->is_disappeared) {
            Point pos1 = robot.value()->pose.pos * scale + offset;
            draw_list->AddCircleFilled({pos1.x(), pos1.y()}, Constants::robot_radius() * scale, friend_color);

            auto path = world_model.control_target_builder[robot.index()]->build(world_model).path;
            std::stringstream ss;
            ss.precision(2);
            for (auto p : path) {
                ss << std::fixed << p.x << "\t" << std::fixed << p.y << "\n";
            }
            draw_list->AddText({pos1.x(), pos1.y()}, ImGui::GetColorU32(ImVec4(1.f, 1.f, 1.f, 1.f)), ss.str().c_str());

            Point pos2 = pos1;
            for (auto p : path) {
                pos1 = pos2;
                pos2 = tool::getPoint(p) * scale + offset;
                draw_list->AddLine({pos1.x(), pos1.y()}, {pos2.x(), pos2.y()}, friend_color, 2.f);
                draw_list->AddCircle({pos2.x(), pos2.y()}, Constants::robot_radius() * scale, friend_color);
            }
        }
    }
    for (auto robot : world_model.enemys.robots) {
        if (!robot->is_disappeared) {
            Point pos = robot->pose.pos * scale + offset;
            draw_list->AddCircleFilled({pos.x(), pos.y()}, Constants::robot_radius() * scale, enemy_color);
        }
    }


    Point ball = world_model.ball.pose.pos * scale + offset;
    draw_list->AddCircleFilled({ball.x(), ball.y()}, 0.05 * scale, ImGui::GetColorU32(ImVec4(1.0f, 0.f, 0.f, 1.0f)));
    draw_list->AddCircleFilled({ball.x(), ball.y()}, 0.5 * scale, ImGui::GetColorU32(ImVec4(1.0f, 0.5f, 0.5f, 0.3f)));
    if (world_model.ball.pose.vel.norm() > 0.5f) {
        Point ball_dst = world_model.ball.pose.vel * 30.f * scale + ball;
        draw_list->AddLine({ball.x(), ball.y()}, {ball_dst.x(), ball_dst.y()},
                           ImGui::GetColorU32(ImVec4(1.0f, 0.f, 0.f, 1.0f)), 2.f);
    }
}
