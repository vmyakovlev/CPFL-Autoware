#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <velodyne_pointcloud/point_types.h>

class DenoiseFilter
{
public:
  DenoiseFilter();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher cloud_publisher_;
  ros::Subscriber cloud_subscriber_;
  std::string output_frame_;

  void callback(const PointCloudMsgT::ConstPtr& msg);
};

DenoiseFilter::DenoiseFilter()
  : nh_(), private_nh_("~"), output_frame_("velodyne")
{
  private_nh_.param("output_frame", output_frame_, output_frame_);
  cloud_subscriber_ = nh_.subscribe("points_no_ground", 1, &DenoiseFilter::callback, this);
  cloud_publisher_ = nh_.advertise<PointCloudMsgT>("denoised_points", 1);
}

void DenoiseFilter::callback(const PointCloudMsgT::ConstPtr& msg)
{
  PointCloudT::Ptr cloud_src(new PointCloudT);
  PointCloudT::Ptr cloud_dst(new PointCloudT);

  // Note: If you use kinetic, you can directly receive messages as PointCloutT.
  pcl::fromROSMsg(*msg, *cloud_src);

  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud_src);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_dst);

  cloud_dst->header = pcl_conversions::toPCL(msg->header);
  cloud_dst->header.frame_id = output_frame_;
  cloud_publisher_.publish(cloud_dst);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denoise_filter");
  DenoiseFilter node;
  ros::spin();
  return 0;
}
