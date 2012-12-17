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

from math import *
import copy

from simple_script_server import *
sss = simple_script_server()


if __name__ == "__main__":
	rospy.init_node("test")
#	sss.move("arm","pos1")
#	sss.move("arm","pos2")
#	sss.move("arm",[[-0.54105206811824214, -1.9198621771937625, 2.0071286397934789, 0.0, 1.4311699866353502, 0.087266462599716474]]) # =pos1
	sss.move("arm","pickup")
	sss.move("arm",["intermediate1","intermediate2"])
	sss.move("arm","drop")
	sss.move("arm",["drop2","drop3","drop4","intermediate2","intermediate1","home"])
