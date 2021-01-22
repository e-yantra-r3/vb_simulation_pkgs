#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from std_srvs.srv import Empty

class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name):

		rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

	def go_to_pose(self, arg_pose):

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		self._group.set_pose_target(arg_pose)
		flag_plan = self._group.go(wait=True)  # wait=False for Async Move

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		if (flag_plan == True):
			pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
		else:
			pass
			# rospy.logerr(
			#     '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

		return flag_plan

	
	def hard_go_to_pose(self, arg_pose, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.go_to_pose(arg_pose)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()





	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

	ur5 = Ur5Moveit(sys.argv[1])

	ur5_pose_1 = geometry_msgs.msg.Pose()
	ur5_pose_1.position.x = -0.817261772949
	ur5_pose_1.position.y = -0.109110076352
	ur5_pose_1.position.z = 0.94446979642
	ur5_pose_1.orientation.x = -0.999999995957
	ur5_pose_1.orientation.y = 4.37354574363e-05
	ur5_pose_1.orientation.z = 7.85715579538e-05
	ur5_pose_1.orientation.w = 2.12177767514e-09

	ur5_pose_2 = geometry_msgs.msg.Pose()
	ur5_pose_2.position.x = -0.414925357653
	ur5_pose_2.position.y = 0.284932768677
	ur5_pose_2.position.z = 1.78027849967
	ur5_pose_2.orientation.x = -0.199396929724
	ur5_pose_2.orientation.y = 1.64394297608e-05
	ur5_pose_2.orientation.z = 0.979918803013
	ur5_pose_2.orientation.w = 6.03911583936e-05

	ur5_pose_3 = geometry_msgs.msg.Pose()
	ur5_pose_3.position.x = 0.061218702528
	ur5_pose_3.position.y = 0.150917431354
	ur5_pose_3.position.z = 1.20083763657
	ur5_pose_3.orientation.x = 0.635613875737
	ur5_pose_3.orientation.y = 0.77190802743
	ur5_pose_3.orientation.z = 0.00233308772292
	ur5_pose_3.orientation.w = 0.0121472162087

	# Pick Pose
	ur5_pick_pose = geometry_msgs.msg.Pose()
	ur5_pick_pose.position.x = 0.030
	ur5_pick_pose.position.y = 0.250
	ur5_pick_pose.position.z = 1.890

	# Place Pose
	ur5_place_pose = geometry_msgs.msg.Pose()
	ur5_place_pose.position.x = -0.770
	ur5_place_pose.position.y = -0.062
	ur5_place_pose.position.z = 0.930

	while not rospy.is_shutdown():
		rospy.logwarn("\n\nPose#1")
		ur5.hard_go_to_pose(ur5_pick_pose, 50)
		rospy.sleep(2)

		rospy.logwarn("\n\nPose#2")
		ur5.hard_go_to_pose(ur5_place_pose, 50)
		rospy.sleep(2)

		# rospy.logwarn("\n\nPose#3")
		# ur5.hard_go_to_pose(ur5_pose_3, 50)
		# rospy.sleep(2)

	del ur5


if __name__ == '__main__':
	main()
