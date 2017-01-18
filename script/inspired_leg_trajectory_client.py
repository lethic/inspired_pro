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


    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['hip_r', 'thigh_r', 'shank_r', 'ankle_r', 'hip_l', 'thigh_l', 'shank_l', 'ankle_l', 'waist']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(0.5)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

def main():
    leg = Joint('leg')
    flag = 1
    joint_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    POS_SIT = [0, 0.614, -0.614, 0, 0, -0.614, 0.614, 0, 0]
    POS_STAND = [0, -0.614, 0.614, 0, 0, 0.614, -0.614, 0, 0]

    POS_LEFT = [0.4, 0, 0, 0.4, 0.4, 0.7, -0.7, 0.4, 0]
    POS_RIGHT = [-0.4, -0.7, 0.7, -0.45, -0.4, 0, 0, -0.4, 0]

    leg.move_joint(POS_STAND)
    state = 1

    while 1:
        leg.jta.wait_for_result()
        if state:
            leg.move_joint(POS_LEFT)
            state = 0
        else:
            leg.move_joint(POS_RIGHT)
            state = 1


if __name__ == '__main__':
    rospy.init_node('inspired_leg_trajectory_controller')
    main()

