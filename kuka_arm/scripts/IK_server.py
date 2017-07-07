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
	    print("Begin.")
            joint_trajectory_point = JointTrajectoryPoint()

	
       #Dealing with machine precision:
	   # mp.dps = 35

        # Define DH param and joint angle symbols. a is linklength, d is X-offset, c is twist angle, q is joint angle.
	    q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
	    d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
	    a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
	    c0,c1,c2,c3,c4,c5,c6 = symbols('c0:7')
	    
	    num_joints = 7

	    # Joint angle limits
	    dtr = pi/180.
	    joint_lims = [(-185*dtr, 185*dtr), (-45*dtr, 85*dtr), (-210*dtr, 65*dtr), (-350*dtr, 350*dtr), (-125*dtr, 125*dtr), (-350*dtr, 350*dtr)]      

            # Modified DH param dictionary
	    dh = {c0: 0, a0: 0, d1 : 0.75, q1: q1,
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
	    r = symbols('r')

	    D = Matrix([[cos(q),	-sin(q),         0,	        a],
			[sin(q)*cos(c), cos(q)*cos(c), -sin(c), -sin(c)*d],
			[sin(q)*sin(c), cos(q)*sin(c), cos(c),   cos(c)*d],
			[            0,             0,      0,          1]])

	    #Defining some zero translation elementary rotation matrices so we can get joint reference frames looking the way they should
	    R_x = Matrix([[ 1,              0,      0, 0],
       		          [ 0,        cos(r), -sin(r), 0],
       		          [ 0,        sin(r),  cos(r), 0],
			  [ 0,             0,       0, 1]])

	    R_y = Matrix([[ cos(r),        0,  sin(r), 0],
	                  [       0,        1,      0, 0],
	                  [-sin(r),        0,  cos(r), 0],
	                  [ 0,             0,       0, 1]])

	    R_z = Matrix([[ cos(r), -sin(r),        0, 0],
	                  [ sin(r),  cos(r),        0, 0],
	                  [ 0,              0,      1, 0],
                          [ 0,             0,       0, 1]])

	    # Creating dictionary of transformation matrices
	    T = {}
	    for joint in range(num_joints):
		last = str(joint)
		current = str(joint + 1)
		dhT = D.subs({a : dh[symbols('a' + last)], c : dh[symbols('c' + last)], q : dh[symbols('q' + current)], d : dh[symbols('d' + current)]})
		T[last + '_' + current] =  dhT

		
 	    print("Assigned vars.")

	    #Getting incremental forward transforms:
        T0_2 = simplify(T['0_1'] * T['1_2'])
        T0_3 = simplify(T0_2 * T['2_3'])
        T0_4 = simplify(T0_3 * T['3_4'])
        T0_5 = simplify(T0_4 * T['4_5'])
        T0_6 = simplify(T0_5 * T['5_6'])
        T0_7 = simplify(T0_6 * T['6_7'])	
		
            
        # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z

        p = Matrix([[px],[py],[pz],[1]])

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
        theta1 = arctan(py, px)  #This gets the first joint to orient the robot correctly. Hopefully this is in the joint's limits because otherwise we're kind of out of luck.

	    # The next two thetas are interrelated by an equation: sin(theta2) * length2 + cos(theta3)*length3 = distance, where the lengths refer to the respective link lengths of each. 
	    # There is a constraint that applies to what theta2 and theta3 can be. First, theta2: it needs to end at a point such that the distance of that point from the requested position == length3
	    # Varying theta2 draws a semi-circular arc, along which only a few points are distance 'length3' from the requested point. If I draw a circle around the reqpoint, then any points that intersect with the joint's
	    # semi-circular arc should be good goal points!

	    # First let's rotate the requested point such that it lies in the flat 2D plane of the joint's arc. We can do this by applying our hopefully correct theta1.
        req_p = R_z.subs({r: -1* theta1}) * p
	
        def findCircleIntersect(l1_,l2_, jx_,jz_, px_,pz_):
            l1,l2 = symbols('l1:3')
            jx = symbols('jx')
            jz = symbols('jz')
            px = symbols('px')
            pz = symbols('pz')
            x = symbols('x')
            y = symbols('y')

            if pz_ - jz_ != 0: #I dont want to ever divide by zero, so I use two different calculations depending.

                #I solved the solution for the intersection by hand, but symbolically recreated the important bits here:
                y_ = ( l1**2 - l2**2 - jx**2 + px**2 - jz**2 + pz**2 - 2*x*(px - jx) ) / (  2*(pz - jz)  ) 

                #Which we can substitute into this quadratic formula for x...
                x_plus = (2*jx + sqrt(   (2*jx)**2 - 4*(  jx**2 + y**2 - 2*y*jz + jz**2 - l1**2 )  )  ) / 2
                x_minus =(2*jx - sqrt(   (2*jx)**2 - 4*(  jx**2 + y**2 - 2*y*jz + jz**2 - l1**2 )  )  ) / 2
                formula_plus = x_plus.subs({y: y_})
                formula_minus = x_minus.subs({y:y_})

			   #Now we substitute real values we know to solve for x...
                plus_x = formula_plus.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_})
                minus_x = formula_minus.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_})


			   #And now for y.
                plus_y = y_.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_}).subs({x: plus_x})
                minus_y = y_.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_}).subs({x: minus_x})

            else:
                #the solution for the intersection by hand, but symbolically recreated the important bits here:
                x_ = ( l1**2 - l2**2 - jx**2 + px**2 - jz**2 + pz**2 - 2*y*(pz - jz) ) / (  2*(px - jx)  )

                #Which we can substitute into this quadratic formula for x...
                y_plus = (2*jz + sqrt(   (2*jz)**2 - 4*(  jz**2 + x**2 - 2*x*jx + jx**2 - l1**2 )  )  ) / 2
                y_minus =(2*jz - sqrt(   (2*jz)**2 - 4*(  jz**2 + x**2 - 2*x*jx + jx**2 - l1**2 )  )  ) / 2
                formula_plus = y_plus.subs({x: x_})
                formula_minus = y_minus.subs({x:x_})

                #Now we substitute real values we know to solve for x...
                plus_y = formula_plus.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_})
                minus_y = formula_minus.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_})


                #And now for y.
                plus_x = x_.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_}).subs({y: plus_y})
                minus_x = x_.subs({l1: l1_, l2: l2_, jx : jx_, jz: jz_, px: px_, pz : pz_}).subs({y: minus_y})

		
            return [ (plus_x, plus_y), (minus_x, minus_y)]
		
 	    #Checking angles generated by the intersection coordinates and choosing the one in our workspace.
	    jx, jz = 0.35, 0.75 #canonical joint centers for the kuka arm
	    intersections = findCircleIntersect(dh[a2], dh[a3], jx, jz, req_p[0], req_p[1])

	    print("Found intersections.")

	    theta2 = -arctan(intersections[0][0], intersecitons[0][1])
	    intersection = 0
        if theta2 < joint_lims[1][0] or theta2 > joint_lims[1][1]:
            theta2 = -arctan(intersections[1][0], intersecitons[1][1]) 
            intersection = 1

	    #To get the third angle I just do a simple arctan given the information from the previous step
	    jx, jz = intersections[intersection]
	    theta3 = -arctan(req_p[2] -jz ,  req_p[0] - jx)

	    print("Found first 3 angles.")

	    # Getting orientation angles now. Since R0_6 = Rrpy, R4_6 = inv(R0_3) * r * p * y. I can break those up, and effectively get the numerical value of sines and cosines to get thetas!
	    inv = T0_3.subs({q1: theta1, q2: theta2, q3: theta3}).inv()
	    theta4 = arctan(-(inv * roll)[0][1] , (inv*roll)[0][0])
	    theta5 = arctan(-(inv * pitch)[0][1] , (inv*pitch)[0][0])
	    theta6 = arctan(-(inv * yaw)[0][1] , (inv*yaw)[0][0])

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
