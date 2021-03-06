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
// Created by hans on 2019/08/22.
//

#pragma once

//-------------include----------------//

#include <utils/eigen_adapter.h>
//------------namespace---------------//
//--------------class-----------------//
namespace geometry_msgs {
    template<class ContainerAllocator>
    struct Pose2D_;
    typedef ::geometry_msgs::Pose2D_<std::allocator<void> > Pose2D;
}

struct Pose2D;
struct BallInfo;


namespace tool {
    Segment getSegment(Point base, Point target);

    Point getPoint(const geometry_msgs::Pose2D &pose);

    geometry_msgs::Pose2D getPose2D(const Point &vec);

    geometry_msgs::Pose2D getPose2D(const Pose2D pose);

    Pose2D getPose2D(const geometry_msgs::Pose2D geo_pose);

    Point getPoint(const Eigen::Vector3f &vec3);

    float getDeg(float angle_rad);

    float getRad(float angle_deg);

    float getAngle(Point vec);

    /**
     * -pi~piの範囲にする
     */
    float normalizeAngle(float angle_rad);

    float getAngleDiff(float angle_rad1, float angle_rad2);

    float getIntermediateAngle(float angle_rad1, float angle_rad2);

    Point getVerticalVec(Point v);

    Point getUnitVec(float theta);

    Point getVec(float length, float theta);

    Point getDirectonNorm(Point base, Point target);

    Point getDirectonVec(Point base, Point target);

    float getReachTime(float distance, float v0, float acc, float max_vel = -1.f);

    Point getBallLineClosestPoint(Point from_pos, const BallInfo &ball);

    bool isInOurDefenseArea(Point pos);

    bool isInTheirDefenseArea(Point pos);
}
