#!/usr/bin/env python3

from tkinter import N
import rospy
import numpy as np
import os
import random
import pandas as pd

from q_learning_project.msg import QMatrix
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")
        print("Initializing\n")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Subscribers and publishers for rewards, matrix, and actions
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)
        self.action_publisher = rospy.Publisher('/q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)
        self.q_matrix_publisher = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)

        self.reward = None
        self.waiting_for_reward = False

        #intialize matrix of 0s
        self.Q = [[0 for i in range(9)] for j in range(64)]
        print("initialized")
        self.r = rospy.Rate(0.5)
        rospy.sleep(3)
        self.q_learning_algorithm()
        print("learned")

    def publish_action(self, action_num):
        obj = self.actions[action_num]['object']
        tag = self.actions[action_num]['tag']

        action = RobotMoveObjectToTag(robot_object = obj, tag_id = tag)

        self.action_publisher.publish(action)

    def get_reward(self, data):
        self.reward = data.reward
        # itr_num = data.iteration_num
        self.waiting_for_reward = False
        return 
    
    def q_learning_algorithm(self):
        print("Starting to learn")
        discount = 0.8
        alpha = 1
        #how many times we want no change before converging
        converge = 64 * 32
        state = 0
        t = 0
        no_change = 0
        
        while(no_change < converge):
            print(no_change, converge)
            #possible actions
            candidates = []

            #loop through action matrix 
            for i in range(len(self.action_matrix)):
                pos_action = self.action_matrix[state][i]
                #select valid actions
                if pos_action >= 0:
                    candidates.append(i)
            #state after action, choose one randomly
            
            #if there are no possible actions, end state, go back to origin and contiue algorithm
            if len(candidates) == 0:
                state = 0
            else: 
                new_state = random.choice(candidates)
                #take action
                action_num = int(self.action_matrix[state][new_state])
                
                self.waiting_for_reward = True
                #publish action and wait for reward subscriber to update
                self.publish_action(action_num)
             
                while (self.waiting_for_reward):
                    i = 1

                q = self.Q[state][action_num]
                #max Q(st+1, at)
                max_q = max(self.Q[new_state])

                #update q value step
                new_q = q + alpha * (self.reward + discount * max_q - q)
                t = t + 1
                
                #check if things changed
                if(new_q != q):
                    no_change = 0 
                    self.Q[state][action_num] = new_q
                else:
                    no_change += 1
        
        
        
    def save_q_matrix(self):
        df = pd.DataFrame(self.Q)
        df.to_csv("~/catkin_ws/src/q_learning_project/q_matrix.csv")
        return True

if __name__ == "__main__":
    node = QLearning()
