#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/registration/ndt.h>

static std::string PARENT_FRAME;
static std::string CHILD_FRAME;
static std::string POINTS_TOPIC;
static int SCAN_NUM;
static int WINDOW_SIZE;
static std::string OUTPUT_DIR;
static bool USE_SLAM;

static tf::TransformListener *tf_listener, *tf_listener_gps;
static tf::TransformListener *tf_listener_test;

static std::string filename_map_gps, filename_slam;

static ros::Publisher map_pub;
static ros::Publisher ndt_pose_pub, velodyne_pose_pub;
static geometry_msgs::PoseStamped ndt_pose_msg, velodyne_pose_msg;

static pcl::PointCloud<pcl::PointXYZI> map_gps, map_slam;
static pcl::PointCloud<pcl::PointXYZI> submap;

static std::queue<tf::StampedTransform> queue_transform;
// static double mean_yaw = 0.0;

static std::queue<pcl::PointCloud<pcl::PointXYZI>> queue_submap;

// ndt parameters
static double trans_eps = 0.01;
static double step_size = 0.1;
static double ndt_res = 1.0;
static int max_iter = 100;

static bool isFirstScan = true;

static Eigen::Matrix4f mat_gps_to_base_link, mat_base_link_to_velodyne;
static tf::Transform tf_gps_to_base_link, tf_base_link_to_velodyne;

// tf between map and base_link
static tf::StampedTransform gps_transform, slam_transform, gps_slam_transform;
static tf::StampedTransform previous_gps_transform, previous_slam_transform, previous_gps_slam_transform;

static double previous_x, previous_y, previous_z, previous_roll, previous_pitch, previous_yaw;

double compute_mean_yaw(std::queue<tf::StampedTransform> input)
{
  double sum_yaw = 0.0;
  double mean_yaw = 0.0;
  int size = input.size();
  while(!input.empty())
  {
    double roll, pitch, yaw;
    tf::Matrix3x3(input.front().getRotation()).getRPY(roll, pitch, yaw);
    sum_yaw += yaw;
    input.pop();
  }
  mean_yaw = sum_yaw/size;
  std::cout << "mean_yaw: " << mean_yaw << std::endl;

  return mean_yaw;
}

pcl::PointCloud<pcl::PointXYZI> mergeQueueSubmap(std::queue<pcl::PointCloud<pcl::PointXYZI>> input)
{
  pcl::PointCloud<pcl::PointXYZI> submap;
  while(!input.empty())
  {
    submap+=input.front();
    input.pop();
  }

  return submap;
}

void points_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input)
{
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<pcl::PointXYZI> gps_transformed_scan, submap;

  std_msgs::Header header;
  pcl_conversions::fromPCL(input->header, header);

  // remove points within 1.0m
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = input->begin(); iter != input->end(); iter++)
  {
    p.x = (double)iter->x;
    p.y = (double)iter->y;
    p.z = (double)iter->z;
    p.intensity = (double)iter->intensity;

    double r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0) + pow(p.z, 2.0));
    if (r > 1.0)
    {
      scan.push_back(p);
    }
  }

  // transform scan using gps transform
  if(scan.points.size() > 0) {
    try {
      tf_listener_gps->waitForTransform("map", "base_link", header.stamp, ros::Duration(1));
      tf_listener_gps->lookupTransform("map", "base_link", header.stamp, gps_transform);

      static tf::StampedTransform test;

      tf_listener_test->waitForTransform("map", "velodyne", header.stamp, ros::Duration(1));
      tf_listener_test->lookupTransform("map", "velodyne", header.stamp, test);

      pcl_ros::transformPointCloud(scan, gps_transformed_scan, test);
      tf::Transform tmp = gps_transform * tf_base_link_to_velodyne;


      double ro, pi, ya;
      std::cout << "gps_transform.getOrigin().getX(): " << gps_transform.getOrigin().getX() << std::endl;
      std::cout << "gps_transform.getOrigin().getY(): " << gps_transform.getOrigin().getY() << std::endl;
      std::cout << "gps_transform.getOrigin().getZ(): " << gps_transform.getOrigin().getZ() << std::endl;
      tf::Matrix3x3 tf(gps_transform.getRotation());
      tf.getRPY(ro,pi,ya);
      std::cout << ro << std::endl;
      std::cout << pi << std::endl;
      std::cout << ya << std::endl;

      std::cout << "tmp.getOrigin().getX(): " << tmp.getOrigin().getX() << std::endl;
      std::cout << "tmp.getOrigin().getY(): " << tmp.getOrigin().getY() << std::endl;
      std::cout << "tmp.getOrigin().getZ(): " << tmp.getOrigin().getZ() << std::endl;
      tf::Matrix3x3 tf2(tmp.getRotation());

      tf2.getRPY(ro,pi,ya);
      std::cout << ro << std::endl;
      std::cout << pi << std::endl;
      std::cout << ya << std::endl;

      std::cout << "test.getOrigin().getX(): " << test.getOrigin().getX() << std::endl;
      std::cout << "test.getOrigin().getY(): " << test.getOrigin().getY() << std::endl;
      std::cout << "test.getOrigin().getZ(): " << test.getOrigin().getZ() << std::endl;
      tf::Matrix3x3 tf3(test.getRotation());

      tf3.getRPY(ro,pi,ya);
      std::cout << ro << std::endl;
      std::cout << pi << std::endl;
      std::cout << ya << std::endl;


    }
    catch (tf::TransformException ex) {
      std::cout << "Transform not found" << std::endl;
      return;
    }
  }

  map_gps += gps_transformed_scan;

  // Set log file name.
  std::ofstream ofs_map_gps;
  filename_map_gps = OUTPUT_DIR + "map_gps.csv";
  ofs_map_gps.open(filename_map_gps.c_str(), std::ios::app);

  if (!ofs_map_gps)
  {
    std::cerr << "Could not open " << filename_map_gps << "." << std::endl;
    exit(1);
  }

  for (int i = 0; i < (int)gps_transformed_scan.points.size(); i++)
  {
    ofs_map_gps << std::fixed << std::setprecision(5) << gps_transformed_scan.points[i].x << ","
                << std::fixed << std::setprecision(5) << gps_transformed_scan.points[i].y << ","
                << std::fixed << std::setprecision(5) << gps_transformed_scan.points[i].z << ","
                << gps_transformed_scan.points[i].intensity << std::endl;
  }

  std::cout << "Wrote " << gps_transformed_scan.size() << " points to " << filename_map_gps << "." << std::endl;

  if(isFirstScan == true){
    map_slam += gps_transformed_scan;
    isFirstScan = false;
  }

  // ndt scan matching
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  double gps_x, gps_y, gps_z, gps_roll, gps_pitch, gps_yaw;
  gps_x = gps_transform.getOrigin().x();
  gps_y = gps_transform.getOrigin().y();
  gps_z = gps_transform.getOrigin().z();
  tf::Matrix3x3(gps_transform.getRotation()).getRPY(gps_roll, gps_pitch, gps_yaw);

  std::ofstream ofs_pose_gps;
  std::string filename_pose_gps = OUTPUT_DIR + "pose_gps.csv";
  ofs_pose_gps.open(filename_pose_gps.c_str(), std::ios::app);

  if (!ofs_pose_gps)
  {
    std::cerr << "Could not open " << filename_slam << "." << std::endl;
    exit(1);
  }

  double gps_pose_x, gps_pose_y, gps_pose_z, gps_pose_roll, gps_pose_pitch, gps_pose_yaw;
  gps_pose_x = gps_transform.getOrigin().x();
  gps_pose_y = gps_transform.getOrigin().y();
  gps_pose_z = gps_transform.getOrigin().z();
  tf::Matrix3x3(gps_transform.getRotation()).getRPY(gps_pose_roll, gps_pose_pitch, gps_pose_yaw);

  ofs_pose_gps << gps_transform.stamp_ << ","
               << std::fixed << std::setprecision(5) << gps_pose_x << ","
               << std::fixed << std::setprecision(5) << gps_pose_y << ","
               << std::fixed << std::setprecision(5) << gps_pose_z << ","
               << std::fixed << std::setprecision(5) << gps_pose_roll << ","
               << std::fixed << std::setprecision(5) << gps_pose_pitch << ","
               << std::fixed << std::setprecision(5) << gps_pose_yaw << std::endl;

  if(USE_SLAM == true && map_slam.points.size() != 0) {
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setStepSize(step_size);
    ndt.setResolution(ndt_res);
    ndt.setMaximumIterations(max_iter);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_slam_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_slam));
    ndt.setInputTarget(map_slam_ptr);

    double voxel_leaf_size = 1.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    // apply voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    ndt.setInputSource(filtered_scan_ptr);

    // Initial pose for scan matching
    Eigen::AngleAxisf init_rotation_x(gps_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(gps_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(gps_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(gps_x, gps_y, gps_z);

    Eigen::Matrix4f init_guess =
        (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * mat_base_link_to_velodyne;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt.align(*output_cloud, init_guess);

    bool has_converged = ndt.hasConverged();
    int final_num_iteration = ndt.getFinalNumIteration();
    double fitness_score = ndt.getFitnessScore();
    double transformation_probability = ndt.getTransformationProbability();
    Eigen::Matrix4f t_velodyne = ndt.getFinalTransformation();
    Eigen::Matrix4f t_base_link = t_velodyne * mat_base_link_to_velodyne.inverse();

    tf::Matrix3x3 mat_velodyne, mat_base_link;

    mat_velodyne.setValue(static_cast<double>(t_velodyne(0, 0)), static_cast<double>(t_velodyne(0, 1)),
                          static_cast<double>(t_velodyne(0, 2)), static_cast<double>(t_velodyne(1, 0)),
                          static_cast<double>(t_velodyne(1, 1)), static_cast<double>(t_velodyne(1, 2)),
                          static_cast<double>(t_velodyne(2, 0)), static_cast<double>(t_velodyne(2, 1)),
                          static_cast<double>(t_velodyne(2, 2)));

    mat_base_link.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                           static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                           static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                           static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                           static_cast<double>(t_base_link(2, 2)));

    ndt_pose_msg.header.frame_id = "map";
    ndt_pose_msg.header.stamp = header.stamp;
    ndt_pose_msg.pose.position.x = t_base_link(0,3);
    ndt_pose_msg.pose.position.y = t_base_link(1,3);
    ndt_pose_msg.pose.position.z = t_base_link(2,3);
    tf::Quaternion q;
    mat_base_link.getRotation(q);
    tf::quaternionTFToMsg(q, ndt_pose_msg.pose.orientation);
    ndt_pose_pub.publish(ndt_pose_msg);

    double roll, pitch, yaw;
    mat_base_link.getRPY(roll, pitch, yaw);

    velodyne_pose_msg.header.frame_id = "map";
    velodyne_pose_msg.header.stamp = header.stamp;
    velodyne_pose_msg.pose.position.x = t_velodyne(0,3);
    velodyne_pose_msg.pose.position.y = t_velodyne(1,3);
    velodyne_pose_msg.pose.position.z = t_velodyne(2,3);
    tf::Quaternion q2;
    mat_velodyne.getRotation(q2);
    tf::quaternionTFToMsg(q2, velodyne_pose_msg.pose.orientation);
    velodyne_pose_pub.publish(velodyne_pose_msg);

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Number of map points: " << map_slam_ptr->points.size() << std::endl;
    std::cout << "Number of scan points: " << input->points.size() << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->points.size() << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Number of iteration: " << final_num_iteration << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Transformation Probability: " << transformation_probability << std::endl;
    std::cout << "Final Transformation:" << std::endl;
    std::cout << t_base_link << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr slam_transformed_scan(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(scan, *slam_transformed_scan, t_velodyne);

    std::cout << "yaw - previous_yaw: " << yaw - previous_yaw << std::endl;
    if(abs(yaw - previous_yaw) <= 0.1) {
      map_slam += *slam_transformed_scan;


      std::ofstream ofs_slam;
      filename_slam = OUTPUT_DIR + "map_slam.csv";
      ofs_slam.open(filename_slam.c_str(), std::ios::app);

      if (!ofs_slam) {
        std::cerr << "Could not open " << filename_slam << "." << std::endl;
        exit(1);
      }

      for (int i = 0; i < (int) slam_transformed_scan->points.size(); i++) {
        ofs_slam << std::fixed << std::setprecision(5) << slam_transformed_scan->points[i].x << ","
                 << std::fixed << std::setprecision(5) << slam_transformed_scan->points[i].y << ","
                 << std::fixed << std::setprecision(5) << slam_transformed_scan->points[i].z << ","
                 << slam_transformed_scan->points[i].intensity << std::endl;
      }

      std::cout << "Wrote " << slam_transformed_scan->points.size() << " points to " << filename_slam << "."
                << std::endl;
    }
    std::ofstream ofs_slam_pose;
    std::string filename_slam_pose = OUTPUT_DIR + "slam_pose.csv";
    ofs_slam_pose.open(filename_slam_pose.c_str(), std::ios::app);

    if (!ofs_slam_pose)
    {
      std::cerr << "Could not open " << filename_slam_pose << "." << std::endl;
      exit(1);
    }

    ofs_slam_pose << gps_transform.stamp_ << ","
                 << std::fixed << std::setprecision(5) << ndt_pose_msg.pose.position.x << ","
                 << std::fixed << std::setprecision(5) << ndt_pose_msg.pose.position.y << ","
                 << std::fixed << std::setprecision(5) << ndt_pose_msg.pose.position.z << ","
                 << std::fixed << std::setprecision(5) << roll << ","
                 << std::fixed << std::setprecision(5) << pitch << ","
                 << std::fixed << std::setprecision(5) << yaw << std::endl;

    // publish map
    while ((int) queue_submap.size() > WINDOW_SIZE) {
      queue_submap.pop();
    }
    queue_submap.push(*slam_transformed_scan);

    submap = mergeQueueSubmap(queue_submap);
    submap.header.frame_id = "/map";

    sensor_msgs::PointCloud2::Ptr submap_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(map_slam, *submap_ptr);
    submap_ptr->header.frame_id = "map";
    map_pub.publish(*submap_ptr);

    previous_x = ndt_pose_msg.pose.position.x;
    previous_y = ndt_pose_msg.pose.position.y;
    previous_z = ndt_pose_msg.pose.position.z;
    previous_roll = roll;
    previous_pitch = pitch;
    previous_yaw = yaw;

  }


  /*
  if((int)queue_transform.size() >= WINDOW_SIZE){
    queue_transform.pop();
  }
  queue_transform.push(transform);
  mean_yaw = compute_mean_yaw(queue_transform);
*/

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("parent_frame", PARENT_FRAME);
  private_nh.getParam("child_frame", CHILD_FRAME);
  private_nh.getParam("points_topic", POINTS_TOPIC);
  private_nh.getParam("scan_num", SCAN_NUM);
  private_nh.getParam("window_size", WINDOW_SIZE);
  private_nh.getParam("use_slam", USE_SLAM);
  private_nh.getParam("output_dir", OUTPUT_DIR);

  std::cout << "parent_frame: " << PARENT_FRAME << std::endl;
  std::cout << "child_frame: " << CHILD_FRAME << std::endl;
  std::cout << "points_topic: " << POINTS_TOPIC << std::endl;
  std::cout << "scan_num: " << SCAN_NUM << std::endl;
  std::cout << "window_size: " << WINDOW_SIZE << std::endl;
  std::cout << "use_slam: " << USE_SLAM << std::endl;
  std::cout << "output_dir: " << OUTPUT_DIR << std::endl;

  // transform between gps to base_link
  Eigen::Translation3f translation_gps_to_base_link(0.0, 0.0, 0.0);
  Eigen::AngleAxisf rot_gps_to_base_link_x(M_PI, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_gps_to_base_link_y(0.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_gps_to_base_link_z(0.0, Eigen::Vector3f::UnitZ());
  mat_gps_to_base_link = (translation_gps_to_base_link * rot_gps_to_base_link_z * rot_gps_to_base_link_y * rot_gps_to_base_link_x).matrix();

  tf::Matrix3x3 tf_mat_gps_to_base_link;
  tf_mat_gps_to_base_link.setValue(static_cast<double>(mat_gps_to_base_link(0, 0)), static_cast<double>(mat_gps_to_base_link(0, 1)),
                                   static_cast<double>(mat_gps_to_base_link(0, 2)), static_cast<double>(mat_gps_to_base_link(1, 0)),
                                   static_cast<double>(mat_gps_to_base_link(1, 1)), static_cast<double>(mat_gps_to_base_link(1, 2)),
                                   static_cast<double>(mat_gps_to_base_link(2, 0)), static_cast<double>(mat_gps_to_base_link(2, 1)),
                                   static_cast<double>(mat_gps_to_base_link(2, 2)));

  tf::Quaternion q_gps_to_base_link;
  tf_mat_gps_to_base_link.getRotation(q_gps_to_base_link);

  tf_gps_to_base_link.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf_gps_to_base_link.setRotation(q_gps_to_base_link);

  // transform between base_link to velodyne
  Eigen::Translation3f translation_base_link_to_velodyne(0.0, 0.0, 0.0);
  Eigen::AngleAxisf rot_base_link_to_velodyne_x(0.0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_base_link_to_velodyne_y(M_PI/2.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_base_link_to_velodyne_z(M_PI, Eigen::Vector3f::UnitZ());
  mat_base_link_to_velodyne = (translation_base_link_to_velodyne * rot_base_link_to_velodyne_z * rot_base_link_to_velodyne_y * rot_base_link_to_velodyne_x).matrix();

  tf::Matrix3x3 tf_mat_base_link_to_velodyne;
  tf_mat_base_link_to_velodyne.setValue(static_cast<double>(mat_base_link_to_velodyne(0, 0)), static_cast<double>(mat_base_link_to_velodyne(0, 1)),
                                   static_cast<double>(mat_base_link_to_velodyne(0, 2)), static_cast<double>(mat_base_link_to_velodyne(1, 0)),
                                   static_cast<double>(mat_base_link_to_velodyne(1, 1)), static_cast<double>(mat_base_link_to_velodyne(1, 2)),
                                   static_cast<double>(mat_base_link_to_velodyne(2, 0)), static_cast<double>(mat_base_link_to_velodyne(2, 1)),
                                   static_cast<double>(mat_base_link_to_velodyne(2, 2)));

  tf::Quaternion q_base_link_to_velodyne;
  tf_mat_base_link_to_velodyne.getRotation(q_base_link_to_velodyne);

  tf_base_link_to_velodyne.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf_base_link_to_velodyne.setRotation(q_base_link_to_velodyne);

  tf_listener = new tf::TransformListener();
  tf_listener_gps = new tf::TransformListener();
  tf_listener_test = new tf::TransformListener();


  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_submap", 1000);
  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1000);
  velodyne_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/velodyne_pose", 1000);
  ros::Subscriber points_sub = nh.subscribe(POINTS_TOPIC, 10, points_callback);

  ros::spin();

  return 0;
}
