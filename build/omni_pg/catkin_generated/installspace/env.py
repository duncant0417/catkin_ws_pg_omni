#!/usr/bin/env python2
#################################################################################
#
#################################################################################

from __future__ import print_function
import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

class Env():
    def __init__(self):
        '''Parameters of Odemetry related '''
        self.robotPosition = Pose()
        self.robotPosition_x = 0
        self.robotPosition_y = 0
        self.odom = []
        self.orientation_list = []

        '''Parameters of Lidar related'''
        self.lidar_scan =[]
        self.lidar_range = []

        '''Parameters of Imu related'''
        self.imu_data = []
        self.imu_linearaccel = []
        self.linear_accel_x = 0
        self.linear_accel_y = 0
        self.linear_accel_z = 0

        '''Init cmd_vel .This will get linear and angular velocity with all zero'''
        self.actionVector = Twist()

        '''Ros Init related Function and variables'''
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.get_OdometryData)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.get_LidarData)
        self.sub_imu = rospy.Subscriber("imu", Imu, self.get_imuData)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        '''wait for topic data get ready ,or you will get nothing in the very beginning time'''
        self.wait_topic_ready()

    def wait_topic_ready(self):
        #print(self.odom)
        dataTest = None
        for _ in range(1) :
            while dataTest is None:
                try:
                    dataTest = rospy.wait_for_message('scan', LaserScan, timeout=5)
                    dataTest = rospy.wait_for_message('odom', LaserScan, timeout=5)
                    dataTest = rospy.wait_for_message('imu', LaserScan, timeout=5)
                except:
                    pass  
        #print(self.odom)   
        del dataTest


    ### This is the topic odom's callback.Please call env.position to get position 
    ### ,do not simply call getOdometry
    def get_OdometryData(self, odom_msg):
        self.odom = odom_msg
        self.robotPosition = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation

        self.robotPosition_x = self.robotPosition.x
        self.robotPosition_y = self.robotPosition.y
        self.orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        
    '''The sharp direction of lidar is the first element, 
    and counterclockwise is the second, 3..... element''' 
    ### This is the topic Lidar's callback.
    ### ,do not simply call getOdometry
    def get_LidarData(self, scan_msg):
        self.lidar_scan = scan_msg
        self.lidar_range = scan_msg.ranges


    ### This is the topic imu's callback.
    ### ,do not simply call getOdometry
    def get_imuData(self, imu_msg):
        self.imu_data = imu_msg
        self.imu_linearaccel = self.imu_data.linear_acceleration
        self.linear_accel_x = self.imu_data.linear_acceleration
        self.linear_accel_y = self.imu_data.linear_acceleration
        self.linear_accel_z = self.imu_data.linear_acceleration

    ### action is a 1x2 vector ,
    # action[0] = linear velocity along x direction
    # action[1] = angular velocity around z axis
    def pubAction_x_w(self,action):
        self.actionVector.linear.x = action[0]
        self.actionVector.angular.z = action[1]
        #public the message to topic cmd_vel
        self.pub_cmd_vel.publish(self.actionVector)

    def env_reset_test(self):
        ### self.reset_proxy() will reset gazebo enviroment ,robot will reset to 
        # its default places which is setting in launch file.
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        self.wait_topic_ready()
        






   