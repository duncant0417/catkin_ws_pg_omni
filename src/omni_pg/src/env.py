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


class Env():
    def __init__(self):
        '''variables of Odemetry related '''
        self.robotPosition = Pose()
        self.robotPosition_x = 0
        self.robotPosition_y = 0
        self.odom = []
        self.orientation_list = []


        '''goal position'''
        self.goal_x = 0.75
        self.goal_y = 0.85

        
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
                scan_range[k] = 2.0 
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
        action_angle = 0
        # 0
        if action == 0:
            print("Action Forward")
            vel_x = 0.2
            vel_y = 0.0
            action_angle = 0

        # 90
        elif action == 1:
            print("Action left")
            vel_x = 0.0
            vel_y = 0.2
            action_angle = 90

        # 45
        elif action == 2:
            print("Action left Forward")
            vel_x = 0.2
            vel_y = 0.2
            action_angle = 45

        # -90
        elif action == 3:
            print("Action right")
            vel_x = 0.0
            vel_y = -0.2
            action_angle = -90

        # -45
        elif action == 4:
            print("Action right Forward")
            vel_x = 0.2
            vel_y = -0.2
            action_angle = -45
        
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
        return action_angle


    def getNewState(self):
        return self.getLaserScan()



    def isCollided(self ,new_state):
        min_new_state = min(new_state)
        print("min_new_state",min_new_state)
        if min_new_state <= 0.16:
            return True

        elif min(new_state[1:4]) <=0.17:
            return True

        elif min(new_state[9:12]) <=0.17:
            return True

        elif min(new_state[16:20]) <=0.17:
            return True
    
        else:
            return False



    def isGoal(self ,x ,y):

        if x >= self.goal_x and y <=self.goal_y :
            return True
        else:
            return False


 

    def setReward(self ,action_angle ,prev_state ,new_state ,x ,y):
        #min_new_state = min(new_state)
        
        state_forward = prev_state[0:2]
        state_forward = np.append(state_forward ,prev_state[22])

        state_f_left = prev_state[2:5]
        state_left = prev_state[5:8]

        state_right = prev_state[17:19]
        state_f_right = prev_state[19:22]


        if self.isCollided(new_state) :
            print("Collided detected !")
            done = True
            reward = -10.0
            print("Reward :",reward)
            return done ,reward

        elif self.isGoal(x ,y):
            print("Goal !")
            done = True
            reward = 100.0
            print("Reward :",reward)
            return done ,reward


        else:
            done = False
            if action_angle == 0:
                print("In action forward reward setting")
                print("state_forward" ,state_forward)
                if max(state_forward) < 0.25:
                    reward = -2
                elif max(state_forward) < 0.35:
                    reward = -0.5
                elif max(state_forward) < 0.45:
                    reward = 0
                elif max(state_forward) < 0.55:
                    reward = 0.5
                else:
                    reward = 1

                if min(state_forward) > 0.45:
                    reward += 2
                elif min(state_forward) > 0.25:
                    reward += 0
                else:
                    reward += -2
                

            elif action_angle == 90:
                print("In action left reward setting")
                print("state_left" ,state_left)
                if max(state_left) < 0.25:
                    reward = -2
                elif max(state_left) < 0.35:
                    reward = -0.5
                elif max(state_left) < 0.45:
                    reward = 0
                elif max(state_left) < 0.55:
                    reward = 0.5
                else:
                    reward = 1

                if min(state_left) > 0.45:
                    reward += 2
                elif min(state_left) > 0.25:
                    reward += 0
                else:
                    reward += -2

            elif action_angle == 45:
                print("In action left forward reward setting")
                print("state_f_left" ,state_f_left)
                if max(state_f_left) < 0.25:
                    reward = -2
                elif max(state_f_left) < 0.35:
                    reward = -0.5
                elif max(state_f_left) < 0.45:
                    reward = 0
                elif max(state_f_left) < 0.55:
                    reward = 0.5
                else:
                    reward = 1

                if min(state_f_left) > 0.45:
                    reward += 2
                elif min(state_f_left) > 0.25:
                    reward += 0
                else:
                    reward += -2

            elif action_angle == -90:
                print("In action right reward setting")
                print("state_right" ,state_right)
                if max(state_right) < 0.25:
                    reward = -2
                elif max(state_right) < 0.35:
                    reward = -0.5
                elif max(state_right) < 0.45:
                    reward = 0
                elif max(state_right) < 0.55:
                    reward = 0.5
                else:
                    reward = 1

                if min(state_right) > 0.45:
                    reward += 2
                elif min(state_right) > 0.25:
                    reward += 0
                else:
                    reward += -2

            elif action_angle == -45:
                print("In action right forward reward setting")
                print("state_f_right" ,state_f_right)
                if max(state_f_right) < 0.25:
                    reward = -2
                elif max(state_f_right) < 0.35:
                    reward = -0.5
                elif max(state_f_right) < 0.45:
                    reward = 0
                elif max(state_f_right) < 0.55:
                    reward = 0.5
                else:
                    reward = 1

                if min(state_f_right) > 0.45:
                    reward += 2
                elif min(state_f_right) > 0.25:
                    reward += 0
                else:
                    reward += -2

        
        print("Reward :",reward)
        return done ,reward


    def step(self ,prev_state ,action):

        action_angle = self.takeAction(action)

        new_state = self.getNewState()

        self.prev_robot_x ,self.prev_robot_y = self.robot_x ,self.robot_y

        '''get robot's x y coordination'''
        self.robot_x ,self.robot_y = self.getRobotStatus()
        print("robot_x :",self.robot_x)
        print("robot_y :",self.robot_y)

        done ,reward = self.setReward(action_angle ,prev_state ,new_state ,self.robot_x ,self.robot_y)

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
        






   