#include "oculus_rviz_plugins/ogre_oculus.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace OVR;
namespace
{
  const int LOOP_RATE = 60;
  ros::Publisher oculus_pose_pub;

  ovrPosef currerntPose;
  Ogre::Quaternion currentOrient;

  int main (int argc, char **argv)
  {
    ros::init(argc,argv,"pub_oculus");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    oculus_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/oculus_pose",1000);

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok())
      {
	currerntPose = oculus_rviz_plugins::Oculus::getPosition();
	currentOrient = oculus_rviz_plugins::Oculus::getOrientation();
	
	geometry_msgs::PoseStamped oculus_pose_msg;
	//	oculu_pose_msg.header.frame_id = ;
	//	oculu_pose_msg.header.stamp = ;
	oculus_pose_msg.pose.position.x = currentPose.Position.x;
	oculus_pose_msg.pose.position.y = currentPose.Position.y;
	oculus_pose_msg.pose.position.z = currentPose.Position.z;
	oculus_pose_msg.pose.orientation.x = currentOrient.x;
	oculus_pose_msg.pose.orientation.y = currentOrient.y;
	oculus_pose_msg.pose.orientation.z = currentOrient.z;
	oculus_pose_msg.pose.orientation.w = currentOrient.w;
	
	oculus_pose_pub.publish(oculus_pose_msg);
	
	ros::spinOnce(); 

	//	loop_rate.sleep();
      }
    return 0;
  }
}
