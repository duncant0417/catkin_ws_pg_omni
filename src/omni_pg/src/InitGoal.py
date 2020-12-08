#!/usr/bin/env python
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


import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class Respawn():
    def __init__(self):
        # os.path.realpath(__file__) will get the current path of this file
        self.CurrentPath = os.path.dirname(os.path.realpath(__file__))
        self.goal_modelPath = self.CurrentPath.replace('omni_pg/src',
                            'omni_3wd/models/goal_box/model.sdf')

        ### Open goal model and them read it into 'goal_model'
        self.fd = open(self.goal_modelPath, 'r')
        self.goal_model = self.fd.read()
        self.goal_position = Pose()

        # Initialize the location of goal
        self.init_goal_x = 0.8 
        self.init_goal_y = 0.5

        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.goal_modelName = 'goal'

        ### Subscript models' states from gazebo
        self.gaebo_model_states = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model_msg):
        self.check_model = False
        #print(model_msg)
        #print(model_msg.name)
        for i in range(len(model_msg.name)):
            if model_msg.name[i] == "goal":
                self.check_model = True


    ### As the function name ,spawn the goal model
    def spawn_goalModel(self):
        ### Wait for Service 'gazebo/spawn_sdf_model' ready
        rospy.wait_for_service('gazebo/spawn_sdf_model')

        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(self.goal_modelName, self.goal_model, 'robotos_name_space', self.goal_position, "world")
        rospy.loginfo("Goal position : %.2f, %.2f", 
                        self.goal_position.position.x,
                        self.goal_position.position.y)


    def delete_goalModel(self):
        rospy.wait_for_service('gazebo/delete_model')

        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        del_model_prox(self.goal_modelName)
