#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import os
import pandas as pd
import numpy as np

import moveit_commander

import math
from math import radians

from q_learning_project.msg import RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

kp = 0.05

class Action:

    def __init__(self):

        self.time_to_drop = False
        
        #intialize this node
        rospy.init_node("robot_action")

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        self.image_init = False
        print("not yet initialized")

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
            Image, self.image_callback)


        # subscribe to the robot's Laser scan data stream
        self.image_sub = rospy.Subscriber('scan',
            LaserScan, self.laser_scan)
        
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('cmd_vel', 
            Twist, queue_size=10)
        
        # Publisher for actions
        self.action_publisher = rospy.Publisher('/q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)

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
        self.color = ""
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
        
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        #read in q_matrix
        self.q_matrix = pd.read_csv("~/catkin_ws/src/q_learning_project/q_matrix.csv").iloc[:,1:].to_numpy()
        #policy of best actions intialized, tested
        self.get_policy()


        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

        self.min_distance = 0.22
        self.min_drop_distance = 0.4

        while not self.image_init:
            print("waiting for init")
        else:
            print("intialized")
            self.do_actions()

    
    def get_policy(self):
        #initialize policy array
        self.policy = []
        state = 0

        while True: 
            #returns action of first highest value
            values = self.q_matrix[state]
            this_action = np.argmax(values)
            value = np.max(values)
            #if the value of the next action is 0, policy complete
            if value == 0:
                break
            self.policy.append(this_action)
            #get new state from action matrix
            new_states = self.action_matrix[state]
            state = np.where(new_states == this_action)[-1]
            
    
    def publish_action(self, action_num):
        obj = self.actions[action_num]['object']
        tag = self.actions[action_num]['tag']

        action = RobotMoveObjectToTag(robot_object = obj, tag_id = tag)

        self.action_publisher.publish(action)
    
    
    #image callback function
    def image_callback(self, msg):
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.image_init = True

    #laser scan callback function
    def laser_scan(self, data):
        scan = np.array(data.ranges)
        #if value is 0, set to 60 because that's outside of range
        scan[scan == 0] = 60
        self.scan = scan

    def find_and_face_color(self):  
        #print("finding color")  
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        #print("There's an image")
        
        #define hsv values of different colors
        bounds = {"pink": {"lower": numpy.array([155,50, 155]),
                            "upper" : numpy.array([170, 255, 255])},
                "green": {"lower": numpy.array([35,50, 155]),
                            "upper" : numpy.array([50, 255, 255])},
                "blue": {"lower": numpy.array([85,50, 155]),
                            "upper" : numpy.array([105, 255, 255])},                   
                }
        

        # this erases all pixels that aren't yellow
        mask = cv2.inRange(hsv, bounds[self.color]['lower'], bounds[self.color]['upper'])

        # this limits our search scope to only view a slice of the image near the ground
        h, w, d = self.image.shape
        #search_top = int(3*h/4)
        #search_bot = int(3*h/4 + 20)
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0

        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        cx = np.NAN
        cy = np.NAN
        # if there are any color pixels found
        if M['m00'] > 0:
                    # center of the color pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # a red circle is visualized in the debugging window to indicate
            # the center point of the yellow pixels
            # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
            cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)

            # TODO: based on the location of the line (approximated
            #       by the center of the yellow pixels), implement
            #       proportional control to have the robot follow
            #       the yellow line

        #how left is the dot from the center of the image 
        rotFactor = -0.005
        if(not np.isnan(cx)):
            offCenter = cx - w/2
            #if facing robot with margin of error
            if offCenter in range(-2, 2):
                self.inFront = True
            myAngular = Vector3(0,0, rotFactor * offCenter)
        else:
            myAngular = Vector3(0,0, 0.1)
            offCenter = 0
            #print("nothing found")
        
        #speed at which to turn
        #print("now moving")
        #print(offCenter)
        
        
        my_twist = Twist(
            #move forward at constant speed
            #linear = Vector3(.05, 0, 0),
            angular = myAngular
        )
        # allow the publisher enough time to set up before publishing the first msg
        #rospy.sleep(0.1)
        #print("slept")

        # publish the message
        self.robot_movement_pub.publish(my_twist)

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        #cv2.imshow("window", image)
        #cv2.waitKey(3)


    def find_and_face_ar(self):
        h, w, d = self.image.shape
        #turn the image into a grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_points = cv2.aruco.detectMarkers(gray,
        self.aruco_dict)
        tag_index = np.where(ids == self.ar)[-1]
        tag_center = np.NAN
        #if found the tag
        if(tag_index.size > 0):
            tag_index = tag_index[0]
            #this gives me four by 2 array of coordinates
            these_corners = corners[tag_index][0]
            #return the x values of the tag
            tag_xs = these_corners[:,0]
            tag_center = np.average(tag_xs)
        #how left is the dot from the center of the image 
        rotFactor = -0.005
        if(not np.isnan(tag_center)):
            offCenter = tag_center - w/2
            #if facing robot with margin of error
            if offCenter in range(-2, 2):
                self.inFrontAR = True
            myAngular = Vector3(0,0, rotFactor * offCenter)
            #myLinear = Vector3(0, 0, 0)
        else:
            myAngular = Vector3(0,0, 0.1)
            #myLinear = Vector3(0,0,0)
            offCenter = 0
            #print("nothing found")
        
        #speed at which to turn
        #print("now moving")
        #print(offCenter)
        
        my_twist = Twist(
            #move forward at constant speed
            linear = Vector3(0, 0, 0),
            angular = myAngular
        )
        # allow the publisher enough time to set up before publishing the first msg
        #rospy.sleep(0.1)
        #print("slept")

        # publish the message
        self.robot_movement_pub.publish(my_twist)


    # Uses proportional control to drive to the target based on self.scan 
    def drive_to_target(self, code):
        
        self.set_movement_arm()
        m = 100

        twist = Twist(
            linear = Vector3(),
            angular = Vector3()
        )

        min_dist = 0.5

        if code == "tube":
            print("driving to tube")
            min_dist = self.min_distance
            bound1, bound2 = -10, 11
        elif code == "ar":
            min_dist = self.min_drop_distance
            bound1, bound2 = -2, 3
        else:
            print("drive_to_target_error")
            return
        
        cnt = 0
        print("ready to move")
        while m > min_dist:
            cnt += 1

            scan = list(self.scan)
            arr = scan[bound1:] + scan[:bound2]
            m = min(arr)

            index = arr.index(m)

            #-9 instead of -10 to compensate for listing to the right
            index -= -(bound1 + 1)

            angular = (kp * index / 2)

            twist.linear.x = 0.05
            twist.angular.z = angular

            rospy.sleep(0.1)
            self.robot_movement_pub.publish(twist)
        
        print("done with while")

        twist.linear.x = 0
        twist.angular.z = 0
        self.robot_movement_pub.publish(twist)
        rospy.sleep(1)
        


    def set_arm(self, goal):
        goal = map(math.radians, goal)
        self.move_group_arm.go(goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(2)

    def set_gripper(self, goal):
        self.move_group_gripper.go(goal, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(2)
        
    def reset(self):
        goal = [0,0,0,0]
        goal = map(math.radians, goal)
        self.move_group_arm.go(goal, wait=True)
        self.move_group_arm.stop()

        gripper_joint_goal = [0.0, 0.0]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(2)
        print("reset")

    def set_movement_arm(self):
        gripper_joint_goal = [0.018, 0.018]
        self.set_gripper(gripper_joint_goal)

        goal = [0,5,0,0]
        self.set_arm(goal)
        print("gripper in position")
        rospy.sleep(2)


    # [1,55,-30,-23] are angles in degrees
    # Gripper can go to 0
    def i_lift_things_up(self):
        print("lifting")

        gripper_joint_goal = [-0.007, -0.007]
        self.set_gripper(gripper_joint_goal)
        goal = [0,-70,0,0]
        self.set_arm(goal)
        rospy.sleep(2)
        print("lifted")
        return True

    def drop_arm(self):

        goal = [0,0,0,0]
        self.set_arm(goal)
        rospy.sleep(5)
        self.time_to_drop = True
    
    def and_put_them_down(self):
        print("putting")
        self.drop_arm()

        while not self.time_to_drop:
            i = 1
        print("arm put")
        # gripper_joint_goal = [0.01, 0.01]
        # self.set_gripper(gripper_joint_goal)
        # print("put")
        self.time_to_drop = False
        print("done")
        return True

    def throw_it_back(self):
        twist = Twist(
            linear = Vector3(),
            angular = Vector3()
        )

        twist.linear.x = -0.1
        self.robot_movement_pub.publish(twist)
        rospy.sleep(1.5)
        twist.linear.x = 0
        self.robot_movement_pub.publish(twist)
        rospy.sleep(1)

    def do_actions(self):
        self.throw_it_back()
        self.drive_to_target("tube")
        self.i_lift_things_up()
        self.and_put_them_down()
        return

        #loop through the policy
        for action_num in self.policy:

            #publish the action
            self.publish_action(action_num)
            self.color = self.actions[action_num]['object']
            self.ar = self.actions[action_num]['tag']
            print("looking for")
            print(self.color)
            self.inFront = False
            while(not self.inFront):
                self.find_and_face_color()
            print("ready for next step")
            rospy.sleep(1)

            #Goes to and picks up dumbbell
            self.drive_to_target("tube")
            self.i_lift_things_up()


            rospy.sleep(1)
            print(self.ar)
            self.inFrontAR = False
            while(not self.inFrontAR):
                self.find_and_face_ar()
            print("ready for next step")
            rospy.sleep(2)

            # Goes to AR and drops dumbbell
            self.drive_to_target("ar")
            rospy.sleep(2)
            self.and_put_them_down()
            self.throw_it_back()

    def run(self):
            rospy.spin()
            
if __name__ == '__main__':

    follower = Action()
    follower.run()
