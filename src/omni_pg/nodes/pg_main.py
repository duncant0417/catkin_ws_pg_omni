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
from keras.utils import plot_model


class POLICY:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.gamma = 0.9
        self.learning_rate = 0.001
        self.states = []
        self.gradients = []
        self.rewards = []
        self.probs = []
        self.model = self._build_model()
        self.model.summary()



    def _build_model(self):
        model = Sequential()


        ##model.add(Dense(128, activation='relu', init='he_uniform' ,input_shape=[self.state_size],))
        ##model.add(Dense(128, activation='relu', init='he_uniform'))
        ##model.add(Dense(self.action_size, activation='softmax'))

        model.add(Dense(128, kernel_initializer="he_uniform", activation="relu", input_shape=[24]))
        model.add(Dense(128, kernel_initializer="he_uniform", activation="relu"))
        model.add(Dense(self.action_size, activation='softmax'))

        opt = Adam(lr=self.learning_rate)
        model.compile(loss='categorical_crossentropy', optimizer=opt)
        plot_model(model, to_file='model.png', show_shapes=True)
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
        aprob = np.reshape(aprob, (self.action_size,))
        self.probs.append(aprob)

        prob = aprob / np.sum(aprob)
        prob = np.reshape(prob, (self.action_size,))

        action = np.random.choice(self.action_size, 1, p=prob)[0]

        return action, prob



    def discount_rewards(self, rewards):
        print("rewards" ,rewards)
        #print("rewards.shape" ,rewards.shape)
        
        discounted_rewards = np.zeros_like(rewards)
        R = 0
        for t in reversed(range(0, rewards.size)):
            #print("     rewards[%d][0] : %d"%(t ,rewards[t][0]))
            R = R * self.gamma + rewards[t][0]
            discounted_rewards[t][0] = R
            #print("discounted_rewards[%d][0] : %d"%(t ,discounted_rewards[t][0]))

        print("discounted_rewards" ,discounted_rewards)
        return discounted_rewards



    def train(self):
        print("In train")

        gradients = np.vstack(self.gradients)
        rewards = np.vstack(self.rewards)


        rewards = self.discount_rewards(rewards)

        rewards = (rewards - np.mean(rewards)) / (np.std(rewards) + 1e-7)

        gradients *= rewards
        #print("gradients" ,gradients)

        X = np.squeeze(np.vstack([self.states]) ,axis = 1)
        Y = self.probs + self.learning_rate * np.vstack([gradients])

        '''
        # state input dim needs to be (n ,24) : ex (1 ,24) (2 ,24)
        # prob input dim needs to be (n ,8) : ex (1 ,8) (2 ,8)

        # self.state.shape = (1, 1, 24)
        # X.shape = (1, 24)
        '''
        loss = self.model.train_on_batch(X, Y)
        print("                                         loss :",loss)
        self.states, self.probs, self.gradients, self.rewards = [], [], [], []
        self.model.summary()



    def load(self, name):
        self.model.load_weights(name)



    def save(self, name):
        self.model.save_weights(name)


N_STATE = 24
N_ACTION = 5 


if __name__ == '__main__':
    '''Init ROS node'''
    rospy.init_node('pg_main')
    env = Env()
    state = env.reset()





    agent = POLICY(N_STATE ,N_ACTION)
    agent.load('ten.h5')
    
    episode =0

    for episode in range(50000):
        score =0


        
        step = 0
        for _ in range(1000) :
            done =False            
            print("step :",step)
            print("******************************************************************************")
            step += 1
            print("prev state :" ,state)

            action ,prob = agent.select_action(state)
            _state ,reward ,done = env.step(state ,action)

            agent.memorize(state ,action ,prob ,reward)

            
            score += reward
            print("Reward for this step :",reward)
            print("done :",done)

            if step >= 1000:
                done = True

            # update old state
            state = _state

            if done :
                episode += 1
                state = env.reset()
                if episode < 50000:
                    agent.train()
                    print('Episode: %d - Score: %f.' % (episode, score))
                    
                    if episode > 1 and episode % 10 == 0:
                        agent.save('ten.h5')

                    if episode > 1 and episode % 100 == 0:
                        agent.save('hun.h5')

                    if episode > 1 and episode % 1000 == 0:
                        agent.save('thoudsen.h5')

                    if episode > 1 and episode % 5000 == 0:
                        agent.save('fivethoudsen.h5')

                    if episode > 1 and episode % 10000 == 0:
                        agent.save('tenthoudsen.h5')
                
                break




