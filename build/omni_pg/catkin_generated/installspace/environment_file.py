#!/usr/bin/env python2
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #
from __future__ import print_function
import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

class Env():
    def __init__(self):
        self.robotPosition = Pose()
        self.robotPosition_x = 0
        self.robotPosition_y = 0
        self.orientation_list = []

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

    ### This is the topic odom's callback.Please call env.position to get position 
    ### ,do not simply call getOdometry
    def getOdometry(self, odom):
        self.robotPosition = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        self.robotPosition_x = self.robotPosition.x
        self.robotPosition_y = self.robotPosition.y
        self.orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        


   