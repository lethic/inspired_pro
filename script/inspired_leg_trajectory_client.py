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
        #print 'Points number: ', len(self.goal.trajectory.points)
        self.jta.send_goal_and_wait(self.goal)
        self.goal.trajectory.points[:] = []

def main():
    leg = Joint('leg')
    flag = 1
    joint_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    POS_SIT = [0, 0.614, -0.614, 0, 0, -0.614, 0.614, 0, 0]
    POS_STAND = [0, -0.65, 0.65, 0, 0, 0.65, -0.65, 0, 0]



    #left shift
    M_L_S = [0.2, 0, 0, 0.3, 0.3, 0, 0, 0.3, 0]
    #right leg up
    M_R_U = [0, 0.7, 0, 0, 0, 0, 0, 0, 0]
    #left shift back to middle and left leg down
    M_FL_S =[-0.45, 0, 0, -0.35, -0.35, 0, 0.7, -0.35, 0]

    #right shift
    M_R_S = [-0.3, 0, 0, -0.3, -0.4, 0, 0, -0.3, 0]
    #left leg up
    M_L_U = [0, 0, 0, 0, 0, -0.7, 0, 0, 0]
    #right shift back to middle and right leg down
    M_FR_S =  [0.35, 0, -0.7, 0.35, 0.45, 0, 0, 0.35, 0]
    #move amount
#    DIFF = 0.3
#    DIFF_M = 0.7
#
#    #left shift
#    M_L_S = [DIFF+0.2, 0, 0, DIFF, DIFF, DIFF_M, 0, DIFF, 0]
#    #right leg thigh up
#    M_R_U = [0, DIFF_M, DIFF_M, 0, 0, 0, 0, 0, 0]
#    #left shift back to middle and left leg shank down
#    M_FL_S =[-(DIFF+0.2), 0, 0, -DIFF, -DIFF, 0, DIFF_M, -DIFF, 0]
#
#    #right shift
#    M_R_S = [-DIFF, -DIFF_M, 0, -DIFF, -(DIFF+0.2), 0, 0, -DIFF, 0]
#    #left leg up
#    M_L_U = [0, 0, 0, 0, 0, -DIFF_M, -DIFF_M, 0, 0]
#    #right shift back to middle and right leg down
#    M_FR_S =  [DIFF, 0, -DIFF_M, DIFF, DIFF+0.2, 0, 0, DIFF, 0]


    joint_pos = POS_STAND
    #leg.add_point(joint_pos, 0.2)
    #joint_pos = map(sum, zip(joint_pos, M_L_S))
    #leg.add_point(joint_pos, 0.5)
    #joint_pos = map(sum, zip(joint_pos, M_R_U))
    #leg.add_point(joint_pos, 1.0)
    #joint_pos = map(sum, zip(joint_pos, M_FL_S))
    leg.add_point(joint_pos, 0.5)
#    joint_pos = map(sum, zip(joint_pos,[0,0,-DIFF_M,0,0,-DIFF_M,0,0,0])) #left leg front, right leg back in the beginning
#    leg.add_point(joint_pos, 0.5)
    leg.move_joint()
    state = 1

    POS_TIME = 0.4

    while 1:
        leg.jta.wait_for_result()
        if state:
            joint_pos = POS_STAND
            joint_pos = map(sum, zip(joint_pos, M_L_S))
            leg.add_point(joint_pos, POS_TIME)
            joint_pos = map(sum, zip(joint_pos, M_R_U))
            leg.add_point(joint_pos, POS_TIME + 0.2)
           # joint_pos = map(sum, zip(joint_pos, M_FL_S))
           # leg.add_point(joint_pos, POS_TIME * 3)
            leg.move_joint()
            state = 0
        else:
            joint_pos = POS_STAND
            joint_pos = map(sum, zip(joint_pos, M_R_S))
            leg.add_point(joint_pos, POS_TIME)
            joint_pos = map(sum, zip(joint_pos, M_L_U))
            leg.add_point(joint_pos, POS_TIME + 0.2)
           # joint_pos = map(sum, zip(joint_pos, M_FR_S))
           # leg.add_point(joint_pos, POS_TIME * 3)
            leg.move_joint()
            state = 1

if __name__ == '__main__':
    rospy.init_node('inspired_leg_trajectory_controller')
    main()

