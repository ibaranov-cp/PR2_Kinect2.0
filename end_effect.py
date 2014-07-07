#!/usr/bin/env python

"""
This software is released under a BSD license:

Copyright (c) 2014, Clearpath Robotics. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


"""
   @file end_effect.py
   @author Ilia Baranov
   @date July 25th, 2014
   @brief ROS node to wrap the nodelet for standalone rosrun execution
"""

import rospy
import math
import time
from util import *
from matrix44 import *
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import GripperCommand

# Gripper init
grip_effort = 100
r_grip = GripperCommand()
r_grip.max_effort = grip_effort
r_grip.position = 0 # 0 - 0.086
l_grip = GripperCommand()
l_grip.max_effort = grip_effort
l_grip.position = 0

#Arm init
r_goal = JointTrajectory()
r_goal.joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
r_goal.points = [JointTrajectoryPoint()]
r_goal.points[0].velocities = [0.0]*7
r_goal.points[0].accelerations = []
r_goal.points[0].positions = [0.0]*7
l_goal = JointTrajectory()
l_goal.joint_names = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
l_goal.points = [JointTrajectoryPoint()]
l_goal.points[0].velocities = [0.0]*7
l_goal.points[0].accelerations = []
l_goal.points[0].positions = [0.0]*7
# this position is arms straight forwards

# Increase this value to create smoother response
avg_period = 6.0

skel_data = [[0 for xyz in xrange(3)] for elements in xrange(27)]
avg_skel_data = [[[0 for xyz in xrange(3)] for elements in xrange(27)] for avg_time in xrange(int(avg_period))]
offset_skel = [[0 for xyz in xrange(3)] for elements in xrange(25)]
r_joint_commands = [0.0001]*7
l_joint_commands = [0.0001]*7

#Kinect joint names 0-24
kinect_joint_names = ['spine_base','spine_mid','neck','head','shoulder_left','elbow_left','wrist_left','hand_left','shoulder_right','elbow_right','wrist_right','hand_right',
    'hip_left','knee_left','ankle_left','foot_left','hip_right','knee_right','ankle_right','foot_right','spine_shoulder','hand_tip_left','thumb_left',
    'hand_tip_right','thumb_right']

def l_x(x):
    global skel_data
    for i in xrange(27):
        skel_data[i][0]=x.data[i]

def l_y(y):
    global skel_data
    for i in xrange(27):
        skel_data[i][1]=y.data[i]

def l_z(z): 
    global skel_data
    for i in xrange(27):
        skel_data[i][2]=z.data[i]

def angle_3d(A,B):
    val = (A[0]*B[0]+A[1]*B[1]+A[2]*B[2])/(math.sqrt(A[0]*A[0]+A[1]*A[1]+A[2]*A[2])*math.sqrt(B[0]*B[0]+B[1]*B[1]+B[2]*B[2]))
    return math.acos(val)


def talker():
    global r_goal
    r_pub = rospy.Publisher('r_arm_controller/command', JointTrajectory)
    l_pub = rospy.Publisher('l_arm_controller/command', JointTrajectory)
    r_grip_pub = rospy.Publisher('r_gripper_controller/command',GripperCommand)
    l_grip_pub = rospy.Publisher('l_gripper_controller/command', GripperCommand)
    rospy.Subscriber("Position_X", Float32MultiArray, l_x)
    rospy.Subscriber("Position_Y", Float32MultiArray, l_y)
    rospy.Subscriber("Position_Z", Float32MultiArray, l_z)
    rospy.init_node('tester', anonymous=True)
    r = rospy.Rate(30) # 30hz to match kinect data
    i_avg = 0

    while not rospy.is_shutdown():

	    #Create moving average
        for i in xrange(27):
            for j in xrange(3):
                avg_skel_data[i_avg][i][j] = skel_data[i][j]
        i_avg += 1
        if (i_avg > avg_period-1): i_avg = 0
        
        #Average skeleton data over averaging period
        for i in xrange(27):
            for j in xrange(3):
                skel_data[i][j] = 0
                for t in xrange(int(avg_period)):
                    skel_data[i][j] += avg_skel_data[t][i][j]
                skel_data[i][j] /= avg_period

        #print "r_grip: " + str(skel_data[26][0]) + " l_grip: " + str(skel_data[25][0])

        if (skel_data[26][0] >= 2.5):
            if r_grip.position == 0.086: print "Closing r_gripper"
            r_grip.position = 0
        if (skel_data[26][0] == 2):
            if r_grip.position == 0: print "Openning r_gripper"
            r_grip.position = 0.086

        r_grip_pub.publish(r_grip)
        
        if (skel_data[25][0] >= 2.5): 
            if l_grip.position == 0.086: print "Closing l_gripper"
            l_grip.position = 0
        if (skel_data[25][0] == 2):
            if l_grip.position == 0: print "Openning l_gripper"
            l_grip.position = 0.086

        l_grip_pub.publish(l_grip)

        #Create body coordinate frame, (0,0,0) at Kinect point 20, leave orientation in Kinect frame     
        #Z gets smaller closer to Kinect
        #Y gets smaller moving down to floor
        #X gets smaller moving left (in body frame)  
        for i in xrange(25):
            for j in xrange(3):
                offset_skel[i][j]= skel_data[i][j] - skel_data[20][j]

        # LEFT ARM------------------------
        #l_shoulder_pan_joint
        l_pan_lift = Vector3.from_points(offset_skel[4],offset_skel[5])
        l_joint_commands[0] = math.atan2(l_pan_lift.x, l_pan_lift.z)
        l_joint_commands[0] += math.pi

        #l_shoulder_lift_joint
        l_joint_commands[1] = math.atan2(l_pan_lift.x, l_pan_lift.y)
        l_joint_commands[1]  = -l_joint_commands[1] - math.pi/2
        
        #l_elbow_flex_joint
        l_joint_commands[3] = -angle_3d(Vector3.from_points(offset_skel[4],offset_skel[5]),Vector3.from_points(offset_skel[5],offset_skel[6]))
        
        #if elbow is not bent, assume arm roll is in zero orientation
        #l_upper_arm_roll_joint
        if l_joint_commands[3] >= 0.25: l_joint_commands[2] = 0
        else:
            shoulder_roll = Vector3.from_points(offset_skel[5],offset_skel[6])
            l_joint_commands[2] = -math.atan2(shoulder_roll.z, shoulder_roll.y)

        #l_forearm_roll_joint
        forearm = Vector3.from_points(offset_skel[21],offset_skel[22])      #Rachet control for forearm is too touchy!
        mul = forearm.cross(Vector3.from_points(offset_skel[7],offset_skel[22]))
        mod = math.atan2(forearm.y, forearm.z)*(-mul[1]/mul[1])
        l_joint_commands[4] = mod

        #l_wrist_flex_joint
        l_joint_commands[5] = -angle_3d(Vector3.from_points(offset_skel[6],offset_skel[7]),Vector3.from_points(offset_skel[7],offset_skel[21]))

        #l_wrist_roll_joint
        l_joint_commands[6] = 0 # Lock wrist roll, too unnatural for human users
        # LEFT ARM------------------------

        # RIGHT ARM------------------------
        #r_shoulder_pan_joint
        pan_lift = Vector3.from_points(offset_skel[8],offset_skel[9])
        r_joint_commands[0] = math.atan2(pan_lift.x, pan_lift.z)
        # Limit joint overdrive
        if r_joint_commands[0] < -1: r_joint_commands[0] = math.pi
        r_joint_commands[0] -= math.pi

        #r_shoulder_lift_joint
        r_joint_commands[1] = math.atan2(pan_lift.x, pan_lift.y)
        r_joint_commands[1] -= math.pi/2
        
        #r_elbow_flex_joint
        r_joint_commands[3] = -angle_3d(Vector3.from_points(offset_skel[8],offset_skel[9]),Vector3.from_points(offset_skel[9],offset_skel[10]))

        #if elbow is not bent, assume arm roll is in zero orientation
        #r_upper_arm_roll_joint
        if r_joint_commands[3] >= 0.25: r_joint_commands[2] = 0
        else:
            shoulder_roll = Vector3.from_points(offset_skel[9],offset_skel[10])
            r_joint_commands[2] = math.atan2(shoulder_roll.z, shoulder_roll.y)

        #r_forearm_roll_joint
        forearm = Vector3.from_points(offset_skel[23],offset_skel[24])      #Rachet control for forearm is too touchy!
        mul = forearm.cross(Vector3.from_points(offset_skel[11],offset_skel[24]))
        mod = math.atan2(forearm.y, forearm.z)*(-mul[1]/mul[1])
        r_joint_commands[4] = -mod

        #r_wrist_flex_joint
        r_joint_commands[5] = -angle_3d(Vector3.from_points(offset_skel[10],offset_skel[11]),Vector3.from_points(offset_skel[11],offset_skel[23]))

        #r_wrist_roll_joint
        r_joint_commands[6] = 0 # Lock wrist roll, too unnatural for human users
        # RIGHT ARM------------------------

        # SEND commands, left in this form for ease of offsetting
        r_goal.points[0].positions[0] = r_joint_commands[0] # shoulder pan
        r_goal.points[0].positions[1] = r_joint_commands[1] # shoulder tilt
        r_goal.points[0].positions[2] = r_joint_commands[2]
        r_goal.points[0].positions[3] = r_joint_commands[3]
        r_goal.points[0].positions[4] = r_joint_commands[4]
        r_goal.points[0].positions[5] = r_joint_commands[5]
        r_goal.points[0].positions[6] = r_joint_commands[6]

        l_goal.points[0].positions[0] = l_joint_commands[0] # shoulder pan
        l_goal.points[0].positions[1] = l_joint_commands[1] # shoulder tilt
        l_goal.points[0].positions[2] = l_joint_commands[2]
        l_goal.points[0].positions[3] = l_joint_commands[3]
        l_goal.points[0].positions[4] = l_joint_commands[4]
        l_goal.points[0].positions[5] = l_joint_commands[5]
        l_goal.points[0].positions[6] = l_joint_commands[6]
        

        r_goal.header.stamp = rospy.get_rostime()
        l_goal.header.stamp = rospy.get_rostime()
        #rospy.loginfo(goal)

        r_pub.publish(r_goal)
        l_pub.publish(l_goal)
        
        r.sleep()

if __name__ == '__main__':
    print "Starting"    
    time.sleep(5)
    try:
        talker()
    except rospy.ROSInterruptException: pass
