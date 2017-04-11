#!/usr/bin/python2

import rospy
import roslib
roslib.load_manifest('inspired_pro')
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

import Tkinter as tk

class Joint:
    def __init__(self, motor_name, motor_list):
        self.name = motor_name
        self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        self.goal = FollowJointTrajectoryGoal()
        #self.goal.trajectory.joint_names = ['hip_r', 'thigh_r', 'shank_r', 'ankle_r', 'hip_l', 'thigh_l', 'shank_l', 'ankle_l', 'waist']
        self.goal.trajectory.joint_names = motor_list

    def add_point(self, angles, seconds):
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(seconds)
        self.goal.trajectory.points.append(point)

    def move_joint(self):
        #print 'Points number: ', len(self.goal.trajectory.points)
        self.jta.send_goal_and_wait(self.goal)
        self.goal.trajectory.points[:] = []

class pubGUI:
    def __init__(self, master, motor_list, INIT_POSE, joints):
        self.master = master
        self.servo_names = motor_list
        self.servo_pos = [0] * len(motor_list)
        self.servo_scales = []
        self.joints = joints
        for r, servo_name in enumerate(self.servo_names):
            tk.Label(text = servo_name, width = 10).grid(row = r, column = 0)
            servo_scale = tk.Scale(self.servo_pos[r], orient = tk.HORIZONTAL, width = 10, length = 400, from_ = -1.7, to = 1.7, resolution = 0.01)
            servo_scale.set(INIT_POSE[r])
            servo_scale.grid(row = r, column = 1)
            self.servo_scales.append(servo_scale)
    def gui_move(self):
        for i, servo_name in enumerate(self.servo_names):
            self.servo_pos[i] = self.servo_scales[i].get()
        self.joints.add_point(self.servo_pos, 0.5)
        self.joints.move_joint()
        print self.servo_pos
        self.master.after(100, self.gui_move)

def main():
    root = tk.Tk()
    rospy.init_node('gui_leg_traj_cli')
    motors_list = ['hip_r', 'thigh_r', 'shank_r', 'ankle_r', 'hip_l', 'thigh_l', 'shank_l', 'ankle_l', 'waist']
    INIT_POSE = [0, -0.5, 0.5, 0, 0, 0.5, -0.5, 0, 0]
    leg = Joint('leg', motors_list)
    app = pubGUI(root, motors_list, INIT_POSE, leg)
    root.after(0, app.gui_move)
    root.mainloop()

if __name__ == "__main__":
    main()

