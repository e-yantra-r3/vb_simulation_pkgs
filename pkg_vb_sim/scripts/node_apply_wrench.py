#! /usr/bin/env python

import rospy

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import ConveyorBeltControlRequest
from pkg_vb_sim.srv import ConveyorBeltControlResponse

from pkg_vb_sim.msg import LogicalCameraImage

from gazebo_msgs.srv import ApplyBodyWrench, GetModelProperties, GetWorldProperties, SetModelState
from geometry_msgs.msg import Wrench


class ConveyorBelt():

	# Constructor
	def __init__(self):
		rospy.loginfo('\033[94m' + " >>> Conveyor Belt Power Apply Wrench init done." + '\033[0m')

	def apply_force(self, arg_body_name, arg_x, arg_y, arg_z):
		success = True
		apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
		body_name = arg_body_name
		wrench = Wrench()
		wrench.force.x = arg_x
		wrench.force.y = arg_y
		wrench.force.z = arg_z

		try:
			resp1 = apply_body_wrench(body_name, "", None, wrench, rospy.Time.from_sec(0), rospy.Duration.from_sec(1.0))
		except rospy.ServiceException:
			success = False
		if success:
			if not resp1.success:
				success = False


	# Destructor
	def __del__(self):
		rospy.loginfo('\033[94m' + " >>> Conveyor Belt Power delete." + '\033[0m')


def main():
	rospy.init_node('node_apply_wrench')

	warehouse_conveyor = ConveyorBelt()

	while not rospy.is_shutdown():
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("conveyor_belt::conveyor_belt_moving::belt", 0, 0, 9.8)

		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen00::link", 0, 0, 0.000000001)
		
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen01::link", 0, 0, 0.000000001)
		
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen02::link", 0, 0, 0.000000001)
		
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen10::link", 0, 0, 0.000000001)
		
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen11::link", 0, 0, 0.000000001)
		
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen12::link", 0, 0, 0.000000001)

		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen20::link", 0, 0, 0.000000001)
		
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen21::link", 0, 0, 0.000000001)
		
		rospy.sleep(0.1)
		warehouse_conveyor.apply_force("packagen22::link", 0, 0, 0.000000001)

		rospy.sleep(1)

	rospy.spin()


if __name__ == "__main__":
	main()
