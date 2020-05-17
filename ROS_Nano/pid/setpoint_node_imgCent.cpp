/***************************************************************************/ /**
 * \file setpoint_node.cpp
 *
 * \brief Node that publishes time-varying setpoint values
 * \author Paul Bouchier
 * \date January 9, 2016
 *
 * \section license License (BSD-3)
 * Copyright (c) 2016, Paul Bouchier
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * - Neither the name of Willow Garage, Inc. nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <ros/console.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "setpoint_node");
  ROS_INFO("Starting setpoint publisher");
  ros::NodeHandle setpoint_node;

  while (ros::ok() && ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Setpoint_node spinning, waiting for time to become non-zero");
    sleep(1);
  }
	
	//std::string camRes;
	int camRes;
	std_msgs::Float64 imgCentre;
	while (ros::ok() && ros::param::get("/zed/zed_node/general/resolution",camRes) == 0) {
		ROS_INFO("Cannot obtain camera resolution. Is camera connected?");
		sleep(1);
	} 
	
	ROS_INFO("Camera resolution parameter set to mode %d",camRes);

	switch(camRes){
		case 0:
			imgCentre.data = 2200 / 2;
			break;
		case 1:
			imgCentre.data = 1920 / 2;
			break;
		case 2:
			imgCentre.data = 1280 / 2;
			break;
		case 3:
			imgCentre.data = 672 / 2;
			break;
	}
	
  std_msgs::Float64 setpoint;
  setpoint.data = imgCentre.data;
  ros::Publisher setpoint_pub = setpoint_node.advertise<std_msgs::Float64>("setpoint", 1, true);
	setpoint_pub.publish(setpoint);
	ros::Rate loop_rate(0.2);
	while (ros::ok())
	{
		loop_rate.sleep();
  }
 
}
