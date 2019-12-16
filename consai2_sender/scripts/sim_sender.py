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
#!/usr/bin/env python2
# coding: UTF-8

import rospy
import socket
import math

from proto import grSim_Packet_pb2
from consai2_msgs.msg import RobotCommands, ReplaceBall, ReplaceBall, Replacements

class SimSender(object):
    def __init__(self):
        self._host = rospy.get_param('consai2_description/grsim_addr', '127.0.0.1')
        self._port = rospy.get_param('consai2_description/grsim_port', 20011)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._sub_commands = rospy.Subscriber(
                'consai2_control/robot_commands',
                RobotCommands,
                self._send_commands,
                queue_size = 10)

        self._sub_replacement = rospy.Subscriber(
                'sim_sender/replacements', Replacements, self._send_replacements,
                queue_size = 1)

        self._MAX_KICK_SPEED = 8.0 # m/s


    def _send_commands(self, msg):
        packet = grSim_Packet_pb2.grSim_Packet()
        packet.commands.timestamp = msg.header.stamp.to_sec()
        packet.commands.isteamyellow = msg.is_yellow

        for command in msg.commands:
            packet_command = packet.commands.robot_commands.add()

            # ロボットID
            packet_command.id = command.robot_id

            # 走行速度
            packet_command.veltangent = command.vel_surge if not math.isnan(command.vel_surge) else 0
            packet_command.velnormal = command.vel_sway if not math.isnan(command.vel_sway) else 0
            packet_command.velangular = command.vel_angular if not math.isnan(command.vel_angular) else 0

            # キック速度
            packet_command.kickspeedx = command.kick_power * self._MAX_KICK_SPEED

            # チップキック
            if command.chip_enable:
                packet_command.kickspeedz = packet_command.kickspeedx
            else:
                packet_command.kickspeedz = 0

            # ドリブラー
            if command.dribble_power > 0:
                packet_command.spinner = True
            else:
                packet_command.spinner = False

            # タイヤ個別に速度設定しない
            packet_command.wheelsspeed = False

        message = packet.SerializeToString()
        self._sock.sendto(message, (self._host, self._port))


    def _send_replacements(self, msg):
        packet = grSim_Packet_pb2.grSim_Packet()

        if msg.ball.is_enabled:
            replace_ball = packet.replacement.ball
            replace_ball.x = msg.ball.x
            replace_ball.y = msg.ball.y
            replace_ball.vx = msg.ball.vx
            replace_ball.vy = msg.ball.vy

        for robot in msg.robots:
            replace_robot = packet.replacement.robots.add()
            replace_robot.x = robot.x
            replace_robot.y = robot.y
            replace_robot.dir = robot.dir
            replace_robot.id = robot.id
            replace_robot.yellowteam = robot.yellowteam
            replace_robot.turnon = robot.turnon

        message = packet.SerializeToString()
        self._sock.sendto(message, (self._host, self._port))


if __name__ == '__main__':
    rospy.init_node('sim_sender')

    sender = SimSender()

    rospy.spin()
