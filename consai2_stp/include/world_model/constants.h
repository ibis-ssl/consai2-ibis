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
// Created by hans on 2019/08/17.
//

#pragma once

//-------------include----------------//
#include <consai2_msgs/VisionGeometry.h>
#include <utils/eigen_adapter.h>

//------------namespace---------------//
//--------------class-----------------//

class Constants {
public:
    static void update(const consai2_msgs::VisionGeometry &geometry) {
        half_field_length_() = geometry.field_length * 0.5f;
        field_length_() = geometry.field_length;

        half_field_width_() = geometry.field_width * 0.5f;
        field_width_() = geometry.field_width;

        half_goal_width_() = geometry.goal_width * 0.5f;
        goal_width_() = geometry.goal_width;

        Our_::Goal::upper() << -half_field_length(), half_goal_width();
        Our_::Goal::center() << -half_field_length(), 0.0f;
        Our_::Goal::lower() << -half_field_length(), -half_goal_width();

        Their_::Goal::upper() << half_field_length(), half_goal_width();
        Their_::Goal::center() << half_field_length(), 0.0f;
        Their_::Goal::lower() << half_field_length(), -half_goal_width();

        static std::multimap<std::string, std::function<void(consai2_msgs::FieldLineSegment &)>> penalty_dict = {
                {"LeftPenaltyStretch",            [&](consai2_msgs::FieldLineSegment &seg) {
                    Our_::Penalty::upper_front() << seg.p2_x, seg.p2_y;
                }},
                {"LeftFieldLeftPenaltyStretch",   [&](consai2_msgs::FieldLineSegment &seg) {
                    Our_::Penalty::upper_back() << seg.p1_x, seg.p1_y;
                }},
                {"LeftPenaltyStretch",            [&](consai2_msgs::FieldLineSegment &seg) {
                    Our_::Penalty::lower_front() << seg.p1_x, seg.p1_y;
                }},
                {"LeftFieldRightPenaltyStretch",  [&](consai2_msgs::FieldLineSegment &seg) {
                    Our_::Penalty::lower_back() << seg.p1_x, seg.p1_y;
                }},
                {"RightPenaltyStretch",           [&](consai2_msgs::FieldLineSegment &seg) {
                    Their_::Penalty::upper_front() << seg.p2_x, seg.p2_y;
                }},
                {"RightFieldRightPenaltyStretch", [&](consai2_msgs::FieldLineSegment &seg) {
                    Their_::Penalty::upper_back() << seg.p1_x, seg.p1_y;
                }},
                {"RightPenaltyStretch",           [&](consai2_msgs::FieldLineSegment &seg) {
                    Their_::Penalty::lower_front() << seg.p1_x, seg.p1_y;
                }},
                {"RightFieldLeftPenaltyStretch",  [&](consai2_msgs::FieldLineSegment &seg) {
                    Their_::Penalty::lower_back() << seg.p1_x, seg.p1_y;
                }}
        };
        for (auto l : geometry.field_lines) {
            auto it_range = penalty_dict.equal_range(l.name);
            for (auto it = it_range.first; it != it_range.second; it++) {
                it->second(l);
            }
        }
    }

protected:
    static float &half_field_length_() {
        static float half_length = field_length_() * 0.5f;
        return half_length;
    }

    static float &half_field_width_() {
        static float half_width = field_width_() * 0.5f;
        return half_width;
    }

    static float &half_goal_width_() {
        static float half_goal;
        return half_goal;
    }

    static float &field_length_() {
        static float length = 12.0f;
        return length;
    }

    static float &field_width_() {
        static float width = 9.0f;
        return width;
    }

    static float &goal_width_() {
        static float goal_w;
        return goal_w;
    }

    static float &ball_radius_() {
        static float ball_r = 0.0215f;
        return ball_r;
    }

    static float &robot_radius_() {
        static float robot_r = 0.09f;
        return robot_r;
    }

    class Our_ {
    public:
        class Goal {
        public:
            static Point &upper() {
                static Point pos;
                return pos;
            }

            static Point &center() {
                static Point pos;
                return pos;
            }

            static Point &lower() {
                static Point pos;
                return pos;
            }
        };

        class Penalty {
        public:
            static Point &upper_front() {
                static Point pos;
                return pos;
            }

            static Point &upper_back() {
                static Point pos;
                return pos;
            }

            static Point &lower_front() {
                static Point pos;
                return pos;
            }

            static Point &lower_back() {
                static Point pos;
                return pos;
            }
        };
    };

    class Their_ {
    public:
        class Goal {
        public:
            static Point &upper() {
                static Point pos;
                return pos;
            }

            static Point &center() {
                static Point pos;
                return pos;
            }

            static Point &lower() {
                static Point pos;
                return pos;
            }
        };

        class Penalty {
        public:
            static Point &upper_front() {
                static Point pos;
                return pos;
            }

            static Point &upper_back() {
                static Point pos;
                return pos;
            }

            static Point &lower_front() {
                static Point pos;
                return pos;
            }

            static Point &lower_back() {
                static Point pos;
                return pos;
            }


        };
    };


public:
    class Our {
    public:
        class Goal {
        public:
            static const Point &upper() { return Our_::Goal::upper(); }

            static const Point &center() { return Our_::Goal::center(); }

            static const Point &lower() { return Our_::Goal::lower(); }
        };

        class Penalty {
        public:
            static const Point &upper_front() { return Our_::Penalty::upper_front(); }

            static const Point &upper_back() { return Our_::Penalty::upper_back(); }

            static const Point &lower_front() { return Our_::Penalty::lower_front(); }

            static const Point &lower_back() { return Our_::Penalty::lower_back(); }

            static Box getArea() {
                Box area(upper_front(), lower_back());
                return area;
            }
        };
    };

    class Their {
    public:
        class Goal {
        public:
            static const Point &upper() { return Their_::Goal::upper(); }

            static const Point &center() { return Their_::Goal::center(); }

            static const Point &lower() { return Their_::Goal::lower(); }

        };

        class Penalty {
        public:
            static const Point &upper_front() { return Their_::Penalty::upper_front(); }

            static const Point &upper_back() { return Their_::Penalty::upper_back(); }

            static const Point &lower_front() { return Their_::Penalty::lower_front(); }

            static const Point &lower_back() { return Their_::Penalty::lower_back(); }

            static Box getArea() {
                Box area(upper_front(), lower_back());
                return area;
            }
        };
    };

    static const float &half_field_length() { return half_field_length_(); }

    static const float &half_field_width() { return half_field_width_(); }

    static const float &half_goal_width() { return half_goal_width_(); }

    static const float &field_length() { return field_length_(); }

    static const float &field_width() { return field_width_(); }

    static const float &goal_width() { return goal_width_(); }

    static const float &ball_radius() { return ball_radius_(); }

    static const float &robot_radius() { return robot_radius_(); }

    static constexpr uint8_t max_id() {
        return 12;
    }
};
