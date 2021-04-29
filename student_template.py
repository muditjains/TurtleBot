#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib
import math
import random
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

gazebo_model_states = ModelStates()

def callback(msg):
	global gazebo_model_states
	gazebo_model_states = msg
def yaw_of(object_orientation):
	euler = euler_from_quaternion((object_orientation.x, object_orientation.y,object_orientation.z, object_orientation.w))

	return euler[2]
def goHome(arm):
		arm.set_named_target("home")
		arm.go()
		rospy.sleep(1.0)
def setGripperGoal(gripper, gripper_goal, val):
		gripper_goal.command.position = val
		gripper.send_goal(gripper_goal)
		gripper.wait_for_result(rospy.Duration(1.0))
		rospy.sleep(1)
def getObjectCoordinates(OBJECT_NAME):
	object_index = gazebo_model_states.name.index(OBJECT_NAME)
	object_position = gazebo_model_states.pose[object_index].position
	object_orientation = gazebo_model_states.pose[object_index].orientation
	object_yaw = yaw_of(object_orientation)

	return (object_position, object_orientation, object_yaw)


def moveArm(arm, X, Y, Z, YAW):

	
	target_pose = Pose()
	target_pose.position.x = X
	target_pose.position.y = Y
	target_pose.position.z = Z


	q = quaternion_from_euler(-math.pi, 0.0, YAW)
	target_pose.orientation.x = q[0]
	target_pose.orientation.y = q[1]
	target_pose.orientation.z = q[2]
	target_pose.orientation.w = q[3]
	arm.set_pose_target(target_pose)
	if arm.go() is False:
		print "Failed to handle the object."
	rospy.sleep(1.0)


def main():
	global gazebo_model_states

	OBJECT_NAME = "wood_cube_5cm"   # name of object to grab]
	sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, callback, queue_size=1)

	arm = moveit_commander.MoveGroupCommander("arm")
	arm.set_max_velocity_scaling_factor(0.4)
	gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
	gripper.wait_for_server()
	gripper_goal = GripperCommandGoal()
	gripper_goal.command.max_effort = 4.0
	# setGripperGoal(gripper,gripper_goal, 1.5707)
	rospy.sleep(1.0)
	# setGripperGoal(gripper,gripper_goal, 0)


	while True:
		goHome(arm)
		position, orientation, yaw = getObjectCoordinates(OBJECT_NAME)
		setGripperGoal(gripper,gripper_goal, 1.5707)
		moveArm(arm, position.x, position.y, 0.12, yaw)
		setGripperGoal(gripper,gripper_goal, 0)
		x = input("x ")
		y = input("y ")
		moveArm(arm, x, y, 0.2, yaw)
		setGripperGoal(gripper,gripper_goal, 1.2)





		#########################################
		#		Write your code below			#
		#########################################



if __name__ == '__main__':
	rospy.init_node("pick_and_place_in_gazebo_example")

	try:
		if not rospy.is_shutdown():
			main()
	except rospy.ROSInterruptException:
		pass
