#!/usr/bin/env python


'''
Original Author :
https://github.com/keon/policy-gradient/blob/master/pg.py

'''

from __future__ import print_function
import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32MultiArray
from src.env import Env
from src.InitGoal import Respawn
#from keras.models import Sequential, load_model
#from keras.optimizers import RMSprop
#from keras.layers.core import Dense, Dropout, Activation

from keras.models import Sequential
from keras.layers import Dense, Reshape, Flatten
from keras.optimizers import Adam
from keras.layers.convolutional import Convolution2D


class POLICY:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.gamma = 0.99
        self.learning_rate = 0.001
        self.states = []
        self.gradients = []
        self.rewards = []
        self.probs = []
        self.model = self._build_model()
        self.model.summary()



    def _build_model(self):
        model = Sequential()


        model.add(Dense(128, activation='relu', init='he_uniform' ,input_shape=[self.state_size],))
        model.add(Dense(128, activation='relu', init='he_uniform'))
        model.add(Dense(self.action_size, activation='softmax'))

        opt = Adam(lr=self.learning_rate)
        model.compile(loss='categorical_crossentropy', optimizer=opt)

        return model



    def memorize(self, state, action, prob, reward):
        state = np.reshape(state, (1, 24))

        y = np.zeros([self.action_size])
        y[action] = 1
        self.gradients.append(np.array(y).astype('float32') - prob)
        self.states.append(state)
        self.rewards.append(reward)




    def select_action(self, state):
        state = np.reshape(state, (1, 24))

        aprob = self.model.predict(state) #.flatten()
        aprob = np.reshape(aprob, (8,))
        self.probs.append(aprob)

        prob = aprob / np.sum(aprob)
        prob = np.reshape(prob, (8,))

        action = np.random.choice(self.action_size, 1, p=prob)[0]

        return action, prob



    def discount_rewards(self, rewards):
        discounted_rewards = np.zeros_like(rewards)
        running_add = 0
        for t in reversed(range(0, rewards.size)):
            if rewards[t] != 0:
                running_add = 0
            running_add = running_add * self.gamma + rewards[t]
            discounted_rewards[t] = running_add
        return discounted_rewards



    def train(self):


        gradients = np.vstack(self.gradients)
        rewards = np.vstack(self.rewards)
        rewards = self.discount_rewards(rewards)
        rewards = (rewards - np.mean(rewards)) / (np.std(rewards) + 1e-7)
        gradients *= rewards

        X = np.squeeze(np.vstack([self.states]) ,axis = 1)

        Y = self.probs + self.learning_rate * np.vstack([gradients])

        '''
        # state input dim needs to be (n ,24) : ex (1 ,24) (2 ,24)
        # prob input dim needs to be (n ,8) : ex (1 ,8) (2 ,8)

        # self.state.shape = (1, 1, 24)
        # X.shape = (1, 24)
        '''
        self.model.train_on_batch(X, Y)
        self.states, self.probs, self.gradients, self.rewards = [], [], [], []



    def load(self, name):
        self.model.load_weights(name)



    def save(self, name):
        self.model.save_weights(name)


N_STATE = 24
N_ACTION = 8 


if __name__ == '__main__':
    '''Init ROS node'''
    rospy.init_node('pg_main')
    env = Env()
    state = env.reset()
    print(env.getRobotStatus())

    episode =0


    for episode in range(10000):
        score =0

        
        agent = POLICY(N_STATE ,N_ACTION)
        #agent.load('pong.h5')

        step = 0
        for _ in range(500) :
            
            print("step :",step)
            print("******************************************************************************")
            step += 1

            action ,prob = agent.select_action(state)
            new_state ,reward ,done = env.step(action)
            score += reward
            agent.memorize(state ,action ,prob ,reward)

            print("Reward for this step :",reward)
            print("accumulation of reward :",score)

            if step >= 100:
                done = True


            if done :
                episode += 1
                state = env.reset()
                if episode < 5000:
                    agent.train()
                    print('Episode: %d - Score: %f.' % (episode, score))
                    
                    if episode > 1 and episode % 10 == 0:
                        agent.save('pong.h5')
                
                break



