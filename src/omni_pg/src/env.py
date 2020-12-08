#!/usr/bin/env python

'''
Author : Yi-Ying Lin
'''

from __future__ import print_function
import rospy
import numpy as np
import math
from math import pi
import time
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates

from respawnGoal import Respawn

class Env():
    def __init__(self):
        '''variables of Odemetry related '''
        self.robotPosition = Pose()
        self.robotPosition_x = 0
        self.robotPosition_y = 0
        self.odom = []
        self.orientation_list = []


        '''goal position'''
        self.goal_x = 0.7

        
        '''variables of Lidar related'''
        self.lidar_scan =[]
        self.lidar_range = []


        '''variables of Imu related'''
        self.imu_data = []
        self.imu_linearaccel = []
        self.linear_accel_x = 0
        self.linear_accel_y = 0
        self.linear_accel_z = 0

        '''variables of obstacle related'''


        '''Init cmd_vel .This will get linear and angular velocity with all zero'''
        self.actionVector = Twist()


        '''Ros Init related Function and variables'''
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometryCallback, queue_size=1000)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.getLidarCallback)
        self.sub_imu = rospy.Subscriber("imu", Imu, self.getImuCallback)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.model_states = rospy.Subscriber('gazebo/model_states', ModelStates, self.getModelStatesCallback)        
        self.modelStatus = []
        self.robot_x = -0.8
        self.robot_y = 0.8
        self.prev_robot_x = -0.8
        self.prev_robot_y = 0.8


        '''wait for topic data get ready ,or you will get nothing in the very beginning time'''
        self.wait_topic_ready()


    def getModelStatesCallback(self ,modelStatusMsg):
        self.modelStatus = modelStatusMsg

    def getRobotStatus(self):
        return self.modelStatus.pose[2].position.x ,self.modelStatus.pose[2].position.y


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


    '''
    ### This is the topic odom's callback.Please call env.position to get position 
    ### ,do not simply call getOdometry
    '''
    def getOdometryCallback(self, odom_msg):
        self.odom = odom_msg
        self.robotPosition = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation

        self.robotPosition_x = self.robotPosition.x
        self.robotPosition_y = self.robotPosition.y
        self.orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

    '''
    get position from odom
    '''
    def getRobotPose_x_y(self):
        return self.robotPosition_x ,self.robotPosition_y






    '''
    The sharp direction of lidar is the first element, 
    and counterclockwise is the second, 3..... element

    ### This is the topic Lidar's callback.
    ### ,do not simply call getOdometry
    '''
    def getLidarCallback(self, scan_msg):
        self.lidar_scan = scan_msg
        self.lidar_range = scan_msg.ranges


    def getLaserScan(self):
        scan_range = []
        scan_range = np.array(self.lidar_range)
        k = 0
        for _ in range(24):
            if scan_range[k] == float('inf') :
                scan_range[k] = 100.0 
            k += 1
           
            
        return scan_range


    '''
    ### This is the topic imu's callback.
    ### ,do not simply call getOdometry
    '''
    def getImuCallback(self, imu_msg):
        self.imu_data = imu_msg
        self.imu_linearaccel = self.imu_data.linear_acceleration
        self.linear_accel_x = self.imu_data.linear_acceleration
        self.linear_accel_y = self.imu_data.linear_acceleration
        self.linear_accel_z = self.imu_data.linear_acceleration



    '''
    step related functions for policy gradient
    '''

    '''
    ### action is a 1x2 vector ,
    # action[0] = linear velocity along x direction
    # action[1] = angular velocity around z axis
    '''
    def pubAction_x_w(self,x ,w):
        self.actionVector.linear.x = x
        self.actionVector.angular.z = w

        #public the message to topic cmd_vel
        self.pub_cmd_vel.publish(self.actionVector)


    def pubAction_x_y(self ,vel_x ,vel_y):
        self.actionVector.linear.x = vel_x
        self.actionVector.linear.y = vel_y
        
        #public the message to topic cmd_vel
        self.pub_cmd_vel.publish(self.actionVector)


    def takeAction(self ,action):
        if action == 0:
            vel_x = 0.2
            vel_y = 0.0

        elif action == 1:
            vel_x = 0.0
            vel_y = 0.2

        elif action == 2:
            vel_x = 0.2
            vel_y = 0.2

        elif action == 3:
            vel_x = 0.0
            vel_y = -0.2

        elif action == 4:
            vel_x = 0.2
            vel_y = -0.2
        
        '''
        elif action == 5:
            vel_x = 0.2
            vel_y = -0.2

        elif action == 6:
            vel_x = -0.2
            vel_y = 0.2

        elif action == 7:
            vel_x = -0.2
            vel_y = -0.2 
        '''

        self.pubAction_x_y(vel_x ,vel_y)
        time.sleep(0.2)


    def getNewState(self):
        return self.getLaserScan()



    def isCollided(self ,new_state):
        min_new_state = min(new_state)
        print("min_new_state",min_new_state)
        if min_new_state <= 0.16:
            return True
    
        else:
            return False



    def isGoal(self ,x):

        if x >= self.goal_x :
            return True
        else:
            return False


 

    def setReward(self ,new_state ,x ,prev_x):
        min_new_state = min(new_state)
        if self.isGoal(x):
            print("Goal !")
            done = True
            reward = 100.0

        elif self.isCollided(new_state) :
            print("Collided detected !")
            done = True
            reward = ( (x - self.goal_x)+1.4 ) * 10.0
            reward += -6.0

        else:
            if min_new_state < 0.2 :
                print("To close.... !")
                done = False
                reward = -0.5

            elif x - prev_x > 0:
                print("Continue.... !")
                done = False
                reward = 1.0

            elif x -prev_x < 0:
                done = False
                reward = -1.0

            else:
                done = False
                reward = 0

        print("Reward :",reward)
        return done ,reward


    def step(self ,action):

        self.takeAction(action)

        new_state = self.getNewState()

        self.prev_robot_x ,self.prev_robot_y = self.robot_x ,self.robot_y

        '''get robot's x y coordination'''
        self.robot_x ,self.robot_y = self.getRobotStatus()
        print("robot_x :",self.robot_x)
        print("robot_y :",self.robot_y)

        done ,reward = self.setReward(new_state ,self.robot_x ,self.prev_robot_x)

        return new_state ,float(reward) ,done


    def reset(self):
        ### self.reset_proxy() will reset gazebo enviroment ,robot will reset to 
        # its default places which is setting in launch file.
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        #except (rospy.ServiceException) as _:
        except :
            print("gazebo/reset_simulation service call failed")

        #self.wait_topic_ready()
        #self.pubAction_x_w(0 ,1)
        #time.sleep(0.1)
        #self.pubAction_x_w(0 ,0)
        self.pubAction_x_y(0 ,0)
        time.sleep(2.0)

        # Return state
        return self.getLaserScan()
        






   