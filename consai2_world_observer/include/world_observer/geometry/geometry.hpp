// MIT License
//
// Copyright (c) 2019 SSL-Roots
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
#ifndef _GEOMETRY_HPP_
#define _GEOMETRY_HPP_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <bfl/filter/extendedkalmanfilter.h>

namespace geometry2d
{

//
// 単純型
//

// Poseクラス
// 2次元の姿勢(x, y, theta) を表現します。
class Pose
{
public:
    double x, y, theta;

    Pose();
    Pose(double x, double y, double theta);
    Pose(geometry_msgs::Pose   pose);
    Pose(geometry_msgs::Pose2D pose);

    MatrixWrapper::ColumnVector ToColumnVector();
    geometry_msgs::Pose ToROSPose();
    geometry_msgs::Pose2D ToROSPose2D();
    geometry2d::Pose Transpose(geometry2d::Pose pose);

    // TODO: この辺の関数は Pose()クラスにあるべき
    double GetNorm();
    double GetAngle();
};

class Velocity
{
public:
    double x, y, theta;

    Velocity();
    Velocity(double x, double y, double theta);
    Velocity(geometry_msgs::Twist twist);

    geometry_msgs::Twist ToROSTwist();
    double GetNorm();
    double GetAngle();
};

class Accel
{
public:
    double x, y, theta;

    Accel();
    Accel(double x, double y, double theta);
    Accel(geometry_msgs::Accel accel);

    MatrixWrapper::ColumnVector ToColumnVector();
};


class Point
{
public:
    double x, y;
};

//
// 複合型
//
class Odometry
{
public:
    Pose pose;
    Velocity velocity;

    Odometry();
    Odometry(Pose pose, Velocity velocity);

    nav_msgs::Odometry ToROSOdometry();

    void print();
};

// Utility methods
double YawFromQuaternion(double x, double y, double z, double w);
geometry_msgs::Quaternion QuaternionFromYaw(double theta);
double pi2pi(double rad);

}

#endif //_GEOMETRY_HPP_
