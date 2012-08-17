/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: cob_industrial
 * \note
 *   ROS stack name: cob_industrial
 * \note
 *   ROS package name: cob_kuka_rsi
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Aug 2012
 *
 * \brief
 *   Implementation of ROS node for kuka rsi.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####
//##################

// standard includes
// --

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <brics_actuator/JointVelocities.h>

#include "math.h"
#define RAD_TO_DEG(a) ((a)*180.0/M_PI)
#define DEG_TO_RAD(a) ((a)/180.0*M_PI)

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// own includes
#include <cob_kuka_rsi/RSIConnector.h>

/*!
 * \brief Implementation of ROS node for kuka rsi.
 *
 * Offers velocity interface.
 */
class RSINode
{

public:
  /// create a handle for this node, initialize node
  ros::NodeHandle n_;

  /// declaration of topics to publish
  ros::Publisher topicPub_JointState_;
  ros::Publisher topicPub_ControllerState_;
  ros::Publisher topicPub_OperationMode_;
  ros::Publisher topicPub_Diagnostic_;

  /// declaration of topics to subscribe, callback is called for new messages arriving
  ros::Subscriber topicSub_CommandVel_;

  /// declaration of service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_SetOperationMode_;
  ros::ServiceServer srvServer_Recover_;

  /// handle for RSI
  RSIConnector* rsi_ctrl_;

  /// member variables
  bool initialized_;
  bool error_;
  std::vector<std::string> joint_names_;

  ///Constructor
  RSINode()
  {
  	n_ = ros::NodeHandle("~");

    rsi_ctrl_ = new RSIConnector("192.1.10.1", 49150, true);

    /// implementation of topics to publish
    topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("/joint_states", 1);
    topicPub_ControllerState_ = n_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState> ("state", 1);
    topicPub_OperationMode_ = n_.advertise<std_msgs::String> ("current_operationmode", 1);
    topicPub_Diagnostic_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    /// implementation of topics to subscribe
    topicSub_CommandVel_ = n_.subscribe("command_vel", 1, &RSINode::topicCallback_CommandVel, this);

    /// implementation of service servers
    srvServer_Init_ = n_.advertiseService("init", &RSINode::srvCallback_Init, this);
    srvServer_Recover_ = n_.advertiseService("recover", &RSINode::srvCallback_Recover, this);
    srvServer_SetOperationMode_ = n_.advertiseService("set_operation_mode", &RSINode::srvCallback_SetOperationMode, this);

    initialized_ = false;

    // TODO: get joint_names from parameter server
    joint_names_.push_back("arm_1_joint");
	joint_names_.push_back("arm_2_joint");
	joint_names_.push_back("arm_3_joint");
	joint_names_.push_back("arm_4_joint");
	joint_names_.push_back("arm_5_joint");
	joint_names_.push_back("arm_6_joint");
  }

  /// Destructor
  ~RSINode()
  {
	  rsi_ctrl_->stop();
	  ROS_INFO("RSI closed!");
  }

  /*!
   * \brief Executes the callback from the command_vel topic.
   *
   * Set the current velocity target.
   * \param msg JointVelocities
   */
  void topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg)
  {
	  ROS_DEBUG("Received new velocity command");
	  if (!initialized_)
	  {
			ROS_WARN("Skipping command: RSI not initialized");
			return;
		}


	  /// command velocities to RSI
	  RSIConnector::AxisCorrection cor;


	  // TODO: clean this up
			#define MAX_VEL 0.1

			cor.dA1 = RAD_TO_DEG(msg->velocities[0].value)/IPOC_HZ;
			if(cor.dA1 > MAX_VEL)
			{
			 cor.dA1 = MAX_VEL;
			}
			else if(cor.dA1 < -MAX_VEL)
			{
			 cor.dA1 = -MAX_VEL;
			}

			cor.dA2 = RAD_TO_DEG(msg->velocities[1].value)/IPOC_HZ;
			if(cor.dA2 > MAX_VEL)
			{
			 cor.dA2 = MAX_VEL;
			}
			else if(cor.dA2 < -MAX_VEL)
			{
			 cor.dA2 = -MAX_VEL;
			}

			cor.dA3 = RAD_TO_DEG(msg->velocities[2].value)/IPOC_HZ;
			if(cor.dA3 > MAX_VEL)
			{
			 cor.dA3 = MAX_VEL;
			}
			else if(cor.dA3 < -MAX_VEL)
			{
			 cor.dA3 = -MAX_VEL;
			}

			cor.dA4 = RAD_TO_DEG(msg->velocities[3].value)/IPOC_HZ;
			if(cor.dA4 > MAX_VEL)
			{
			 cor.dA4 = MAX_VEL;
			}
			else if(cor.dA4 < -MAX_VEL)
			{
			 cor.dA4 = -MAX_VEL;
			}

			cor.dA5 = RAD_TO_DEG(msg->velocities[4].value)/IPOC_HZ;
			if(cor.dA5 > MAX_VEL)
			{
			 cor.dA5 = MAX_VEL;
			}
			else if(cor.dA5 < -MAX_VEL)
			{
			 cor.dA5 = -MAX_VEL;
			}


			cor.dA6 = RAD_TO_DEG(msg->velocities[5].value)/IPOC_HZ;
			if(cor.dA6 > MAX_VEL)
			{
			 cor.dA6 = MAX_VEL;
			}
			else if(cor.dA6 < -MAX_VEL)
			{
			 cor.dA6 = -MAX_VEL;
			}


	  rsi_ctrl_->SetAxisCorrection(cor);
	  // TODO
	  // set error_
  }

  /*!
   * \brief Executes the service callback for init.
   *
   * Connects to the hardware and initialized it.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
	  if (!initialized_)
	  {
		  ROS_INFO("Initializing RSI...");

		  // TODO initialize RSI
		  rsi_ctrl_->start();
		  initialized_ = true;
		res.success.data = true;
		res.error_message.data = "text";
	  }
	  else
	  {
		  res.success.data = true;
		  res.error_message.data = "RSI already initialized";
		  ROS_WARN("...initializing RSI not successful. error: %s",res.error_message.data.c_str());
	  }
	  return true;
  }

  /*!
   * \brief Executes the service callback for recover.
   *
   * Recovers the driver after an emergency stop.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
	  ROS_INFO("Recovering RSI...");
	  if (initialized_)
	  {
		  // TODO call RSI->Recover())
		  res.success.data = false;
        	res.error_message.data = "text";
			ROS_ERROR("...recovering RSI successful. error: %s", res.error_message.data.c_str());
	  }
	  else
	  {
		  res.success.data = false;
		  res.error_message.data = "RSI not initialized";
		  ROS_ERROR("...recovering RSInot successful. error: %s",res.error_message.data.c_str());
	  }
	  return true;
  }

   /*!
   * \brief Executes the service callback for SetOperationMode.
   *
   * Sets the driver to different operation modes. Currently only operation_mode=velocity is supported.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res)
  {
	if(req.operation_mode.data != "velocity")
	{
		ROS_WARN("RSI currently only supports velocity commands");
		res.success.data = false;
	}
	else
	{
		res.success.data = true;
	}
	return true;
  }

  /*!
   * \brief Publishes the state of the rsi as ros messages.
   *
   * Published to "/joint_states" as "sensor_msgs/JointState"
   * Published to "state" as "pr2_controllers_msgs/JointTrajectoryState"
   */
  void publishState()
  {
	  if (initialized_)
	  {
		ROS_DEBUG("publish state");

		// TODO update state from RSI
		// set error_
		RSIConnector::RobotPosition pos = rsi_ctrl_->GetRobPos();
		std::vector<double> pos_vec;
		pos_vec.push_back(DEG_TO_RAD(pos.A1));
		pos_vec.push_back(DEG_TO_RAD(pos.A2));
		pos_vec.push_back(DEG_TO_RAD(pos.A3));
		pos_vec.push_back(DEG_TO_RAD(pos.A4));
		pos_vec.push_back(DEG_TO_RAD(pos.A5));
		pos_vec.push_back(DEG_TO_RAD(pos.A6));

		//TODO calculate velocities

		sensor_msgs::JointState joint_state_msg;
		joint_state_msg.header.stamp = ros::Time::now();
		joint_state_msg.name = joint_names_;
		joint_state_msg.position = pos_vec;
		// TODO: fill in velocities joint_state_msg.velocity = ...

		pr2_controllers_msgs::JointTrajectoryControllerState controller_state_msg;
		controller_state_msg.header.stamp = joint_state_msg.header.stamp;
		controller_state_msg.joint_names = joint_state_msg.name;
		controller_state_msg.actual.positions = joint_state_msg.position;
		controller_state_msg.actual.velocities = joint_state_msg.velocity;
		//controller_state_msg.actual.accelerations = ...

		std_msgs::String opmode_msg;
		opmode_msg.data = "velocity";

		/// publishing joint and controller states on topic
		topicPub_JointState_.publish(joint_state_msg);
		topicPub_ControllerState_.publish(controller_state_msg);
		topicPub_OperationMode_.publish(opmode_msg);
	  }

    // publishing diagnotic messages
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);

    // set data to diagnostics
    if(error_)
    {
      diagnostics.status[0].level = 2;
      diagnostics.status[0].name = n_.getNamespace();
      diagnostics.status[0].message =  "text"; // TODO fill in error message
    }
    else
    {
      if (initialized_)
      {
        diagnostics.status[0].level = 0;
        diagnostics.status[0].name = n_.getNamespace();
        diagnostics.status[0].message = "RSI initialized and running";
      }
      else
      {
        diagnostics.status[0].level = 1;
        diagnostics.status[0].name = n_.getNamespace();
        diagnostics.status[0].message = "RSI not initialized";
      }
    }
    // publish diagnostic message
    topicPub_Diagnostic_.publish(diagnostics);
  }
}; //RSINode

/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "kuka_rsi");

	// create RSINode
	RSINode rsi_node;

	/// get main loop parameters
	double frequency;
	if (rsi_node.n_.hasParam("frequency"))
	{
		rsi_node.n_.getParam("frequency", frequency);
	}

	else
	{
		//frequency of driver has to be much higher then controller frequency
		frequency = 10; //Hz
		ROS_WARN("Parameter frequency not available, setting to default value: %f Hz", frequency);
	}

	/// main loop
	ros::Rate loop_rate(frequency); // Hz
	while (rsi_node.n_.ok())
	{
		rsi_node.publishState();

		/// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
