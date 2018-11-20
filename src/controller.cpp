/* Copyright 2018 Zachary Zimits

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright 
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include "Listener.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber scanSub;
  ros::Publisher velPub;
  ros::Rate rate (10);
  Listener listener;
	
  geometry_msgs::Twist driveMsg;
  geometry_msgs::Twist turnMsg;
  driveMsg.linear.x = 0.2;
  turnMsg.angular.z=1;
	
	
  velPub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",10);
  scanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&Listener::processLaserScan, &listener);
  
  while (ros::ok()){
    ros::spinOnce();
	
    if(listener.getClosest()<0.8) {
		ROS_DEBUG_STREAM("Turn: "<<listener.getClosest());
		velPub.publish(turnMsg);
	} else {
		ROS_DEBUG_STREAM("Drive: "<<listener.getClosest());
		velPub.publish(driveMsg);
	rate.sleep();
  
	}
  }
  return 0;
}