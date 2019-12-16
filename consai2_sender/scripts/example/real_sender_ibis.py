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
import math
import serial
import socket
import struct
from consai2_msgs.msg import RobotCommands

check=0

class RealSender(object):
    def __init__(self):
    
        self._MAX_VEL_SURGE = 2.0 # m/s
        self._MAX_VEL_SWAY  = 2.0 # m/s
        self._MAX_VEL_ANGULAR = 2.0*math.pi

        self._sub_commands = rospy.Subscriber(
                'consai2_control/robot_commands',
                RobotCommands,
                self._send,
                queue_size = 10)

    def _send(self, msg):
        global check
        robotid_send=0
        vel_angular_send=0
        vel_angular_send_low=0
        vel_angular_send_high=0
        vel_angular_vision_send=0
        vel_angular_vision_send_low=0
        vel_angular_vision_send_high=0
        vel_angular_consai_send=0
        vel_angular_consai_send_low=0
        vel_angular_consai_send_high=0
        vel_surge_send=0
        vel_surge_send_low=0
        vel_surge_send_high=0
        vel_sway_send=0
        vel_sway_send_low=0
        vel_sway_send_high=0
        kick_power=0.0
        kick_power_send=0
        dribble_power=0.0
        dribble_power_send=0
        robot_x_send=0.0
        robot_x_send_low= 0.0
        robot_x_send_high=0.0
        robot_y_send=0.0
        robot_y_send_low= 0.0
        robot_y_send_high=0.0
        robot_y_target_send=0.0
        robot_y_target_send_low= 0.0
        robot_y_target_send_high= 0.0
        robot_x_target_send=0.0
        robot_x_target_send_low= 0.0
        robot_x_target_send_high= 0.0
        keeper_EN=0

        for command in msg.commands:
    
            # ロボットID
            robotid_send=command.robot_id
    
            #vel_surge
            # -2 ~ 2 -> 0 ~ 32767 ~ 65534
            # -2 -> 0
            # 0 -> 32767
            # 2 -> 65534
            vel_surge_send= int(32767*(command.vel_surge/ self._MAX_VEL_SURGE)+ 32767)
            vel_surge_send_low = vel_surge_send  & 0x00FF    
            vel_surge_send_high= (vel_surge_send & 0xFF00)>>8
            
            #vel_sway
            # -2 ~ 2 -> 0 ~ 32767 ~ 65534
            # -2 -> 0
            # 0 -> 32767
            # 2 -> 65534
            vel_sway_send= int(32767*(command.vel_sway /self._MAX_VEL_SWAY)+ 32767)
            vel_sway_send_low = vel_sway_send  & 0x00FF    
            vel_sway_send_high= (vel_sway_send & 0xFF00)>>8
            
            # 走行角速度
            # -2pi ~ 2pi -> 0 ~ 32767 ~ 65534
            # -2pi -> 0
            # 0 -> 32767
            # 2pi -> 65534
            vel_angular = command.vel_angular
            if math.fabs(vel_angular) > self._MAX_VEL_ANGULAR:
                vel_angular = math.copysign(self._MAX_VEL_ANGULAR, vel_angular)
            # -2pi ~ 2pi -> 0 ~ 32767 ~ 65534
            vel_angular_send=int(32767*(vel_angular/self._MAX_VEL_ANGULAR) + 32767)
            vel_angular_send_low=  vel_angular_send & 0x00FF
            vel_angular_send_high= (vel_angular_send & 0xFF00)>>8

            # 目標角度
            # -pi ~ pi -> 0 ~ 32767 ~ 65534
            # -pi -> 0
            # 0 -> 32767
            # pi -> 65534
            vel_angular_consai = command.target_theta/2
            if math.fabs(vel_angular_consai) > math.pi:
                vel_angular_consai = math.copysign(math.pi, vel_angular_consai)
            # -pi ~ pi -> 0 ~ 32767 ~ 65534
            vel_angular_consai_send=int(32767*(vel_angular_consai/math.pi) + 32767)
            vel_angular_consai_send_low=  vel_angular_consai_send & 0x00FF
            vel_angular_consai_send_high= (vel_angular_consai_send & 0xFF00)>>8

            # Vision角度
            # -pi ~ pi -> 0 ~ 32767 ~ 65534
            # pi -> 0
            # 0 -> 32767
            # pi -> 65534
            vel_angular_vision = command.theta/2
            if math.fabs(vel_angular_vision) > math.pi:
                vel_angular_vision = math.copysign(math.pi, vel_angular_vision)
            # -pi ~ pi -> 0 ~ 32767 ~ 65534
            vel_angular_vision_send=int(32767*(vel_angular_vision/math.pi) + 32767)
            vel_angular_vision_send_low=  vel_angular_vision_send & 0x00FF
            vel_angular_vision_send_high= (vel_angular_vision_send & 0xFF00)>>8

            # ドリブル
            # 0 ~ 1.0 -> 0 ~ 20
            if command.dribble_power > 0:
                dribble_power = command.dribble_power
                if dribble_power > 1.0:
                    dribble_power = 1.0
                elif dribble_power < 0:
                    dribble_power = 0
                dribble_power_send= int(round(20 * dribble_power))

            # キック
            # 0 ~ 1.0 -> 0 ~ 20
            # チップキック有効の場合　0 ~ 1.0 -> 100 ~ 120
            if command.kick_power > 0:
                kick_power = command.kick_power
                if kick_power > 1.0:
                    kick_power = 1.0
                elif kick_power < 0:
                    kick_power = 0
                if(command.chip_enable):
                    kick_power_send = int((round(20 * kick_power)+100))
                else:
                    kick_power_send = int(round(20 * kick_power))

            # キーパーEN
            # 0 or 1
            keeper_EN = command.local_mode_enable

            # ボール座標ｘ  フィールドが60ｍまで対応
            # m => mm
            ball_x = command.ball_x*1000
            # -mm ~ +mm -> 0 ~ 30000 ~ 65534
            ball_x_send=int(ball_x + 30000)
            ball_x_send_low=  ball_x_send & 0x00FF
            ball_x_send_high= (ball_x_send & 0xFF00)>>8

            # ボール座標y
            # m => mm
            ball_y = command.ball_y*1000
            # -mm ~ +mm -> 0 ~ 30000 ~ 65534
            ball_y_send=int(ball_y + 30000)
            ball_y_send_low=  ball_y_send & 0x00FF
            ball_y_send_high= (ball_y_send & 0xFF00)>>8

            # ロボット座標x
            # m => mm
            robot_x = command.pos_x*1000
            # -mm ~ +mm -> 0 ~ 30000 ~ 65534
            robot_x_send=int(robot_x + 30000)
            robot_x_send_low=  robot_x_send & 0x00FF
            robot_x_send_high= (robot_x_send & 0xFF00)>>8

            # ロボット座標y
            # m => mm
            robot_y = command.pos_y*1000
            # -mm ~ +mm -> 0 ~ 30000 ~ 65534
            robot_y_send=int(robot_y + 30000)
            robot_y_send_low=  robot_y_send & 0x00FF
            robot_y_send_high= (robot_y_send & 0xFF00)>>8
            
            # ロボット目標座標x
            # m => mm
            robot_x_target =  command.target_pos_x*1000
            # -mm ~ +mm -> 0 ~ 30000 ~ 65534
            robot_x_target_send=int(robot_x_target + 30000)
            robot_x_target_send_low=  robot_x_target_send & 0x00FF
            robot_x_target_send_high= (robot_x_target_send & 0xFF00)>>8

            # ロボット目標座標y
            # m => mm
            robot_y_target =  command.target_pos_y*1000
            # -mm ~ +mm -> 0 ~ 30000 ~ 65534
            robot_y_target_send=int(robot_y_target + 30000)
            robot_y_target_send_low=  robot_y_target_send & 0x00FF
            robot_y_target_send_high= (robot_y_target_send & 0xFF00)>>8



            if(command.robot_id==0):
                UDP_IP = "192.168.20.100"
                UDP_PORT_send = 12345
            elif(command.robot_id==1):
                UDP_IP = "192.168.20.101"
                UDP_PORT_send = 12345
            elif(command.robot_id==2):
                UDP_IP = "192.168.20.102"
                UDP_PORT_send = 12345
            elif(command.robot_id==3):
                UDP_IP = "192.168.20.103"
                UDP_PORT_send = 12345
            elif(command.robot_id==4):
                UDP_IP = "192.168.20.104"
                UDP_PORT_send = 12345
            elif(command.robot_id==5):
                UDP_IP = "192.168.20.105"
                UDP_PORT_send = 12345
            elif(command.robot_id==7):
                UDP_IP = "192.168.20.106"
                UDP_PORT_send = 12345
            elif(command.robot_id==8):
                UDP_IP = "192.168.20.108"
                UDP_PORT_send = 12345
            elif(command.robot_id==9):
                UDP_IP = "192.168.20.109"
                UDP_PORT_send = 12345
            elif(command.robot_id==10):
                UDP_IP = "192.168.20.110"
                UDP_PORT_send = 12345
            elif(command.robot_id==11):
                UDP_IP = "192.168.20.111"
                UDP_PORT_send = 12345
            elif(command.robot_id==12):
                UDP_IP = "192.168.20.112"
                UDP_PORT_send = 12345
            elif(command.robot_id==13):
                UDP_IP = "192.168.20.113"
                UDP_PORT_send = 12345
            elif(command.robot_id==14):
                UDP_IP = "192.168.20.114"
                UDP_PORT_send = 12345
            elif(command.robot_id==15):
                UDP_IP = "192.168.20.115"
                UDP_PORT_send = 12345


            if(command.robot_id==1):
                rospy.loginfo("id=%d Vx=%.3f Vy=%.3f omega=%.3f vision=%.3f theta=%.3f kick=%.2f chip=%d Dri=%.2f"
                ,command.robot_id,command.vel_surge,command.vel_sway,command.vel_angular,vel_angular_vision,vel_angular_consai,kick_power
                ,command.chip_enable,dribble_power)

                rospy.loginfo(" ball_x=%d ball_y=%d robot_x=%d robot_y=%d target_x=%d target_y=%d check=%d"
                ,ball_x,ball_y,robot_x,robot_y,robot_x_target,robot_y_target,check)
                #rospy.loginfo("=%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x",
                #,vel_surge_send_high,vel_surge_send_low,vel_sway_send_high,vel_sway_send_low,vel_angular_send_high,vel_angular_send_low,
                #vel_angular_vision_send_high,vel_angular_vision_send_low,vel_angular_consai_send_high,vel_angular_consai_send_low,kick_power_send,
                #dribble_power_send,keeper_EN,ball_x_send_high,ball_x_send_low,ball_y_send_high,ball_y_send_low,robot_x_send_high,robot_x_send_low,
                #robot_y_send_high,robot_y_send_low,robot_x_target_send_low,robot_x_target_send_high,robot_y_target_send_low,robot_y_target_send_high,check)
                
            check=check+1
            if(check>250):
                check=0
        
            packed_data = struct.pack('B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B''B'
            ,vel_surge_send_high,vel_surge_send_low,vel_sway_send_high,vel_sway_send_low,vel_angular_send_high,vel_angular_send_low,
            vel_angular_vision_send_high,vel_angular_vision_send_low,vel_angular_consai_send_high,vel_angular_consai_send_low,kick_power_send,
            dribble_power_send,keeper_EN,ball_x_send_high,ball_x_send_low,ball_y_send_high,ball_y_send_low,robot_x_send_high,robot_x_send_low,
            robot_y_send_high,robot_y_send_low,robot_x_target_send_low,robot_x_target_send_high,robot_y_target_send_low,robot_y_target_send_high,check)
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(packed_data, (UDP_IP, UDP_PORT_send))

def main():
    rospy.init_node('real_sender')
    sender = RealSender()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass

