# MIT License
#
# Copyright (c) 2019 SSL-Roots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import random

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D

from actions import tool
import role


class Observer(object):
    _moving_speed_threshold = 1.0
    _moving_speed_hysteresis = 0.3
    _ball_is_moving = False
    _role_is_exist = [False] * (len(role.ROLE_ID) + 1)
    _random_zero_one = random.randint(0,1)

    @classmethod
    def update_ball_is_moving(cls, ball_info):
        velocity = ball_info.velocity
        ball_speed = tool.get_size_from_center(velocity)

        if Observer._ball_is_moving == False and \
                ball_speed > Observer._moving_speed_threshold + Observer._moving_speed_hysteresis:
            Observer._ball_is_moving = True
        elif Observer._ball_is_moving == True and \
                ball_speed < Observer._moving_speed_threshold - Observer._moving_speed_hysteresis:
            Observer._ball_is_moving = False

    @classmethod
    def update_role_is_exist(cls, role_is_exist):
        Observer._role_is_exist = role_is_exist

    @classmethod
    def ball_is_moving(cls):
        return Observer._ball_is_moving

    @classmethod
    def role_is_exist(cls, role_id):
        return Observer._role_is_exist[role_id]

    @classmethod
    def update_random_zero_one(cls):
        Observer._random_zero_one = random.randint(0,1)

    @classmethod
    def random_zero_one(cls):
        return Observer._random_zero_one
