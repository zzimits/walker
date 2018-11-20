#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <algorithm>
#include <iterator>

class Listener {
public:
  float closest;
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
	closest = 10;
	std::vector<float>::iterator ptr;
	std::vector<float> data = scan->ranges;
	for (ptr=data.begin();ptr<data.end();ptr++){
		if (*ptr<closest)
			closest=*ptr;
	}  
	  //closest = *std::min_element(data.begin(),data.end());
  }
	
  float getClosest(){
	  return closest;
  }
};

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
		ROS_INFO_STREAM("Turn: "<<listener.getClosest());
		velPub.publish(turnMsg);
	} else {
		ROS_INFO_STREAM("Drive: "<<listener.getClosest());
		velPub.publish(driveMsg);
	rate.sleep();
  
	}
  }
  return 0;
}