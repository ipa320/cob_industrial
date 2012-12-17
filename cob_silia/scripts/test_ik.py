#!/usr/bin/python

#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_scenarios
# \note
#   ROS package name: cob_generic_states_experimental
#
# \author
#   Ulrich Reiser, email:ulrich.reiser@ipa.fhg.de
#
# \date Date of creation: June 26 2012
#
# \brief
#   Implements generic detection routine which is used in multiple detection states.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################


import roslib
roslib.load_manifest('cob_silia')
import rospy
import sys

from math import *
import copy
import tf

from simple_script_server import *
sss = simple_script_server()

def callIKSolver(current_pose, goal_pose):
	req = GetPositionIKRequest()
	req.ik_request.ik_link_name = "arm_6_link"
	req.ik_request.ik_seed_state.joint_state.position = current_pose
	req.ik_request.pose_stamped = goal_pose
	resp = iks(req)
	result = []
	for o in resp.solution.joint_state.position:
		result.append(o)
	return (result, resp.error_code)

if __name__ == "__main__":
	rospy.init_node("test")
	iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
#	sss.move("arm","pos1")
	object_pose_bl = PoseStamped()
	object_pose_bl.header.stamp = rospy.Time.now()
	object_pose_bl.header.frame_id = "stack_link"
	object_pose_bl.pose.position.x = -0.384
	object_pose_bl.pose.position.y = 0.185
	object_pose_bl.pose.position.z = 1.106
	[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(0, 0, 1.414) # rpy 
	object_pose_bl.pose.orientation.x = new_x
	object_pose_bl.pose.orientation.y = new_y
	object_pose_bl.pose.orientation.z = new_z
	object_pose_bl.pose.orientation.w = new_w

	# calculate ik solutions for pre grasp configuration
	arm_home = rospy.get_param("/script_server/arm/home")
	(arm_conf, error_code) = callIKSolver(arm_home[0], object_pose_bl)
	if(error_code.val != error_code.SUCCESS):
		rospy.logerr("Ik failed")
		sys.exit()

#	object_pose_bl2 = copy.deepcopy(object_pose_bl)
#	object_pose_bl2.pose.position.y = -0.440
#	(arm_conf2, error_code) = callIKSolver(arm_home[0], object_pose_bl2)
#	if(error_code.val != error_code.SUCCESS):
#		rospy.logerr("Ik failed")
#		sys.exit()

#	handle_arm = sss.move("arm", [arm_conf,"intermediate1","intermediate2",arm_conf2])
	handle_arm = sss.move("arm", [arm_conf])
	
	
