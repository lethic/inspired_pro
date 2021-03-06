#!/usr/bin/env python
import roslib
roslib.load_manifest('inspired_pro')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Joint:
    def __init__(self, motor_name):
        self.name = motor_name
        self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.joint_names = ['hip_r', 'thigh_r', 'shank_r', 'ankle_r', 'hip_l', 'thigh_l', 'shank_l', 'ankle_l', 'waist']

    def add_point(self, angles, seconds):
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(seconds)
        self.goal.trajectory.points.append(point)

    def move_joint(self):
        print 'Points number: ', len(self.goal.trajectory.points)
        self.jta.send_goal_and_wait(self.goal)
        self.goal.trajectory.points[:] = []

def main():
    leg = Joint('leg')
    flag = 1
    joint_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    POS_SIT = [0, 0.5, -0.5, 0, 0, -0.5, 0.5, 0, 0]
    POS_STAND = [0, -0.5, 0.5, 0, 0, 0.5, -0.5, 0, 0]



    #left shift
    M_L_S = [0.3, 0, 0, 0.3, 0.3, 0, 0, 0.3, 0]
    #right leg up
    M_R_U = [0, 0.2, 0, 0, 0, 0, 0, 0, 0]
    #right hip
    M_R_H = [-0.1, 0, 0, 0, 0, 0, 0, 0, 0]
    #right leg up again
    M_R_U2 = [0, 0.4, 0, 0, 0, 0, 0, 0, 0]
    #left shift again
    M_L_S2 = [0, 0, 0, 0, 0.1, 0, 0, 0.1, 0]
    #left thigh up
    M_L_U = [0, 0, 0, 0, 0, 0.4, 0, 0, 0]
    #right shank forward
    R_S_F = [0.1, 0, 0.4, 0, 0, 0, 0.3, 0, 0]
    #right leg straigt, left thigh up
    #R_L_S = [0, -0.6, -0.4, 0, 0, -0.4, -0.3, 0, 0]

    #ankles straight
    R_A_S = [-0.3, 0, 0, -0.3, -0.4, 0, 0.3, -0.4, 0]
    #here is[0, 0.1, 0.9, 0, 0, 0.9, 0.1, 0, 0]

    #shift right
    M_R_S = [-0.3, 0, 0, -0.3, -0.3, 0, 0, -0.3, 0]
    #right thigh forward
    R_T_F = [0, -1.0, 0, 0, 0, 0, 0, 0, 0]
    #left thigh up
    L_T_U = [0, 0, 0, 0, 0, -0.5, 0, 0, 0]
    L_H_U = [0, 0, 0, 0, 0.1, 0, 0, 0, 0]
    L_T_U2 = [0, 0, 0, 0, 0, -0.5, 0, 0, 0]
    #left shank forward
    L_S_F = [0, 0, 0, 0, 0, 0, -1.0, 0, 0]
    #right shank forward
    R_S_B = [0, 0, -1.0, 0, 0, 0, 0, 0, 0]
    #shift back
    S_F_B = [0.3, 0, 0, 0.3, 0.2, 0, 0, 0.3, 0]

    #right shift
    #F_R_S = [-0.3, 0, 0, -0.3, -0.3, 0, 0, -0.3, 0]
    #left leg up
    #F_L_U1 = [0, 0, 0, 0, 0, -0.7, -0.5, 0, 0]
    #F_L_U = [0, -0.5, 0.5, 0, 0, 0, 0, 0, 0]


    joint_pos = POS_STAND
    leg.add_point(joint_pos, 0.5)
    leg.move_joint()
    state = 1

    POS_TIME = 0.5

    while 1:
        leg.jta.wait_for_result()
        if state:
            joint_pos = POS_STAND
            joint_pos = map(sum, zip(joint_pos, M_L_S))
            leg.add_point(joint_pos, POS_TIME)
            joint_pos = map(sum, zip(joint_pos, M_R_U))
            leg.add_point(joint_pos, POS_TIME *2)
            joint_pos = map(sum, zip(joint_pos, M_R_H))
            leg.add_point(joint_pos, POS_TIME *3)
            joint_pos = map(sum, zip(joint_pos, M_R_U2))
            leg.add_point(joint_pos, POS_TIME *4)
            joint_pos = map(sum, zip(joint_pos, M_L_S2))
            leg.add_point(joint_pos, POS_TIME *5)
            joint_pos = map(sum, zip(joint_pos, M_L_U))
            leg.add_point(joint_pos, POS_TIME *6)
            joint_pos = map(sum, zip(joint_pos, R_S_F))
            leg.add_point(joint_pos, POS_TIME *7)
            joint_pos = map(sum, zip(joint_pos, R_A_S))
            leg.add_point(joint_pos, POS_TIME *8)
            joint_pos = map(sum, zip(joint_pos, M_R_S))
            leg.add_point(joint_pos, POS_TIME * 9)
            joint_pos = map(sum, zip(joint_pos, R_T_F))
            leg.add_point(joint_pos, POS_TIME *10)
            joint_pos = map(sum, zip(joint_pos, L_T_U))
            leg.add_point(joint_pos, POS_TIME *11)
            joint_pos = map(sum, zip(joint_pos, L_H_U))
            leg.add_point(joint_pos, POS_TIME *12)
            joint_pos = map(sum, zip(joint_pos, L_T_U2))
            leg.add_point(joint_pos, POS_TIME *13)
            joint_pos = map(sum, zip(joint_pos, L_S_F))
            leg.add_point(joint_pos, POS_TIME *14)
            joint_pos = map(sum, zip(joint_pos, R_S_B))
            leg.add_point(joint_pos, POS_TIME *15)
            joint_pos = map(sum, zip(joint_pos, S_F_B))
            leg.add_point(joint_pos, POS_TIME *16)



            leg.move_joint()
            state = 0
            print('finish state 0')
        else:
            state = 1
            print('finish state 1')

if __name__ == '__main__':
    rospy.init_node('inspired_leg_trajectory_controller')
    main()

