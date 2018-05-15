/*
 *  Copyright (c) 2017, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

class LaserScanToPointCloud {

public:
  LaserScanToPointCloud();
  ~LaserScanToPointCloud();

private:
  ros::NodeHandle nh_, private_nh_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tf_listener_;
  ros::Subscriber laser_sub_;
  ros::Publisher cloud_pub_;
  std::string frame_id;
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

LaserScanToPointCloud::LaserScanToPointCloud() : nh_(), private_nh_("~")
{
  private_nh_.param<std::string>("frame_id", frame_id, "scan");
  laser_sub_ = nh_.subscribe("scan", 1, &LaserScanToPointCloud::callback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
}

void LaserScanToPointCloud::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 cloud;
  try
  {
    projector_.transformLaserScanToPointCloud(frame_id, *msg, cloud, tf_listener_);
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR(e.what());
    return;
  }
  cloud_pub_.publish(cloud);
}

LaserScanToPointCloud::~LaserScanToPointCloud()
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_to_pointcloud");
  LaserScanToPointCloud node;
  ros::spin();

  return 0;
}
