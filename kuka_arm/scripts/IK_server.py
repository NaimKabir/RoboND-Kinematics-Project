#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param and joint angle symbols. a is linklength, d is X-offset, c is twist angle, q is joint angle.
	    q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
	    d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
	    a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
	    c0,c1,c2,c3,c4,c5,c6 = symbols('c0:7')
	    
	    num_joints = 7      

            # Modified DH param dictionary
	    dh = {c0: 0, a0: 0, d1 = 0.75, q1: q1,
		  c1: -pi/2, a1: 0.35, d2: 0, q2: q2 - (pi/2),	    
		  c2: 0, a2: 1.25, d3: 0, q3: q3,
		  c3: -pi/2, a3: -0.054, d4: 1.5, q4 : q4,
		  c4: pi/2, a4: 0, d5: 0, q5: q5,                                 
		  c5: -pi/2, a5: 0, d6: 0,  q6: q6,                                
		  c6: 0, a6: 0, d7: 0.303, q7: 0
		  }

	    # Define Modified DH Transformation matrix, with the parameters of the last joint's twist angle (c), last joint's link length (a), this joint's angle (q), 
	    a = symbols('a')
	    c = symbols('c')
	    q = symbols('q')
	    d = symbols('d')

	    D = Matrix([[cos(q),	sin(q),         0,	        a],
			[sin(q)*cos(c), cos(q)*cos(c), -sin(c), -sin(c)*d],
			[sin(q)*sin(c), cos(q)*sin(c), cos(c),   cos(c)*d],
			[            0,             0,      0,          1]])

	    # Creating dictionary of transformation matrices
	    T = {}
	    for joint in range(num_joints):
		last = str(joint)
		current = str(joint + 1)
		T[last + '_' + current] = D.subs({a = dh['a' + last], c = dh['c' + last], q = dh['q' + current], d = dh['d' + current]})
		 
            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method

		


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
