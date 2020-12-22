#!/usr/bin/env python2

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

'''
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x
'''

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Move():
    def __init__(self):
        '''Init cmd_vel .This will get linear and angular velocity with all zero'''
        self.actionVector = Twist()

        '''Ros Init related Function and variables'''
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)


    def pubAction_x_y_w(self ,vel_x ,vel_y ,vel_w):
        self.actionVector.linear.x = vel_x
        self.actionVector.linear.y = vel_y
        self.actionVector.angular.z = w

        #public the message to topic cmd_vel
        self.pub_cmd_vel.publish(self.actionVector)


    def takeAction(self ,action):
        # 0
        if action == 0:
            print("Action Forward")
            vel_x = 0.2
            vel_y = 0.0
            vel_w = 0.0

        # 90
        elif action == 1:
            print("Action left")
            vel_x = 0.0
            vel_y = 0.2
            vel_w = 0.0

        # 45
        elif action == 2:
            print("Action left Forward")
            vel_x = 0.2
            vel_y = 0.2
            vel_w = 0.0

        # -90
        elif action == 3:
            print("Action right")
            vel_x = 0.0
            vel_y = -0.2
            vel_w = 0.0

        # -45
        elif action == 4:
            print("Action right Forward")
            vel_x = 0.2
            vel_y = -0.2
            vel_w = 0.0

        elif action == 5:
            print("Action back")
            vel_x = -0.2
            vel_y = 0.0
            vel_w = 0.0

        elif action == 6:
            print("Action right back")
            vel_x = -0.2
            vel_y = -0.2
            vel_w = 0.0

        elif action == 7:
            print("Action left back")
            vel_x = -0.2
            vel_y = 0.2
            vel_w = 0.0

        elif action == 8:
            print("Action rotate right")
            vel_x = 0.0
            vel_y = 0.0
            vel_w = 1.5
    
        elif action == 9:
            print("Action rotate left")
            vel_x = 0.0
            vel_y = 0.0
            vel_w = -1.5

        elif action == 10:
            print("Stop")
            vel_x = 0.0
            vel_y = 0.0
            vel_w = 0.0           



        self.pubAction_x_y_w(vel_x ,vel_y ,vel_w)


if __name__=="__main__":

  move = Move()
  while(1):
      key = getKey()
      if key == 'w' :
        move.takeAction(0)
      elif key == 'x' :
        move.takeAction(5)

      elif key == 'a' :
        move.takeAction(1)

      elif key == 'd' :
        move.takeAction(3)

      elif key == 'q' :
        move.takeAction(2)

      elif key == 'e' :
        move.takeAction(4)

      elif key == 'z' :
        move.takeAction(7)

      elif key == 'c' :
        move.takeAction(6)

      elif key == 'r' :
        move.takeAction(8)

      elif key == 'f' :
        move.takeAction(9)

      elif key == 's' :
        move.takeAction(10)

      else:
          if (key == '\x03'):
              break