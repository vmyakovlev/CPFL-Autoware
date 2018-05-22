//
// Created by kosuke on 11/29/17.
//
#include <ros/ros.h>

#include <array>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

#include <tf/transform_datatypes.h>

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

#include "naive_L_shape.h"

// using namespace std;
// using namespace pcl;

int g_count = 0;

ClusterFilter::ClusterFilter() {
  // float picScale = 30;
  roi_m_ = 60;
  pic_scale_ = 900 / roi_m_;
  ram_points_ = 80;

  // l-shape fitting params
  slope_dist_ = 2.0;
  num_points_ = 300;

  // float sensor_height_ = 1.73;
  sensor_height_ = 2.35;

  // rule-based filter params
  height_min_ = 1.0;
  // tHeightMin_ = 1.2
  height_max_ = 2.6;
  // tWidthMin_ = 0.5;
  // tWidthMin_ = 0.4;
  width_min_ = 0.25;
  width_max_ = 3.5;
  len_min_ = 0.5;
  len_max_ = 14.0;
  area_max_ = 20.0;
  ratio_min_ = 1.3;
  ratio_max_ = 5.0;
  min_len_ratio_ = 3.0;
  pt_num_per_vol_ = 8; // point c

  sub_cloud_array_ = node_handle_.subscribe("/cloud_clusters", 1,
                                            &ClusterFilter::callBack, this);
  pub_cloud_array_ = node_handle_.advertise<autoware_msgs::CloudClusterArray>(
      "/bbox_cluster_array", 1);
}

void ClusterFilter::callBack(autoware_msgs::CloudClusterArray input) {
  // std::cout << input.clusters[2].centroid_point.point.x<<std::endl;
  autoware_msgs::CloudClusterArray out_cluster;
  getBBoxes(input, out_cluster);
  // std::cout << input.header << std::endl;
  out_cluster.header = input.header;
  pub_cloud_array_.publish(out_cluster);
  // std::cout << input.clusters[2].centroid_point.point.x<<std::endl;
  g_count++;
  std::cout << "Frame " << g_count
            << "------------------------------------------------" << std::endl;
  std::cout << "cluster size putpuy " << out_cluster.clusters.size()
            << std::endl;
}

void ClusterFilter::getPointsInPcFrame(cv::Point2f rect_points[],
                                       std::vector<cv::Point2f> &pc_points,
                                       int offset_x, int offset_y) {
  // loop 4 rect points
  for (int point_i = 0; point_i < 4; point_i++) {
    float pic_x = rect_points[point_i].x;
    float pic_y = rect_points[point_i].y;
    // reverse offset
    float r_offset_x = pic_x - offset_x;
    float r_offset_y = pic_y - offset_y;
    // reverse from image coordinate to eucledian coordinate
    float r_x = r_offset_x;
    float r_y = pic_scale_ * roi_m_ - r_offset_y;
    // reverse to 30mx30m scale
    float rm_x = r_x / pic_scale_;
    float rm_y = r_y / pic_scale_;
    // reverse from (0 < x,y < 30) to (-15 < x,y < 15)
    float pc_x = rm_x - roi_m_ / 2;
    float pc_y = rm_y - roi_m_ / 2;
    cv::Point2f point(pc_x, pc_y);
    pc_points[point_i] = point;
  }
}

bool ClusterFilter::ruleBasedFilter(std::vector<cv::Point2f> pc_points,
                                    float max_z, int num_points) {
  bool is_promising = false;
  // minnimam points thresh
  if (num_points < 100)
    return is_promising;
  // length is longest side of the rectangle while width is the shorter side.
  float width, length, height, area, ratio, mass;

  float x1 = pc_points[0].x;
  float y1 = pc_points[0].y;
  float x2 = pc_points[1].x;
  float y2 = pc_points[1].y;
  float x3 = pc_points[2].x;
  float y3 = pc_points[2].y;

  float dist1 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  float dist2 = sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
  if (dist1 > dist2) {
    length = dist1;
    width = dist2;
  } else {
    length = dist2;
    width = dist1;
  }
  // assuming ground = sensor height
  height = max_z + sensor_height_;
  // assuming right angle
  area = dist1 * dist2;
  mass = area * height;
  ratio = length / width;

  // start rule based filtering
  if (height > height_min_ && height < height_max_) {
    if (width > width_min_ && width < width_max_) {
      if (length > len_min_ && length < len_max_) {
        if (area < area_max_) {
          if (num_points > mass * pt_num_per_vol_) {
            if (length > min_len_ratio_) {
              if (ratio > ratio_min_ && ratio < ratio_max_) {
                is_promising = true;
                return is_promising;
              }
            } else {
              is_promising = true;
              return is_promising;
            }
          }
        }
      }
    }
  } else
    return is_promising;
}

void ClusterFilter::updateCpFromPoints(std::vector<cv::Point2f> pc_points,
                                       autoware_msgs::CloudCluster &cluster) {
  cv::Point2f p1 = pc_points[0];
  cv::Point2f p2 = pc_points[1];
  cv::Point2f p3 = pc_points[2];
  cv::Point2f p4 = pc_points[3];

  double s1 =
      ((p4.x - p2.x) * (p1.y - p2.y) - (p4.y - p2.y) * (p1.x - p2.x)) / 2;
  double s2 =
      ((p4.x - p2.x) * (p2.y - p3.y) - (p4.y - p2.y) * (p2.x - p3.x)) / 2;
  double cx = p1.x + (p3.x - p1.x) * s1 / (s1 + s2);
  double cy = p1.y + (p3.y - p1.y) * s1 / (s1 + s2);

  // cout << cx << " "<< cy << endl;
  cluster.bounding_box.pose.position.x = cx;
  cluster.bounding_box.pose.position.y = cy;
  cluster.bounding_box.pose.position.z = -sensor_height_ / 2;
}

void ClusterFilter::toRightAngleBBox(std::vector<cv::Point2f> &pc_points) {
  cv::Point2f p1 = pc_points[0];
  cv::Point2f p2 = pc_points[1];
  cv::Point2f p3 = pc_points[2];
  cv::Point2f p4 = pc_points[3];

  double vec1x = p2.x - p1.x;
  double vec1y = p2.y - p1.y;
  double vec2x = p3.x - p2.x;
  double vec2y = p3.y - p2.y;

  // from the equation of inner product
  double cos_theta =
      (vec1x * vec2x + vec1y * vec2y) / (sqrt(vec1x * vec1x + vec2x * vec2x) +
                                         sqrt(vec1y * vec1y + vec2y * vec2y));
  double theta = acos(cos_theta);
  double diff_theta = theta - M_PI / 2;

  if (abs(diff_theta) > 0.1) {
    double m1 = vec1y / vec1x;
    double b1 = p3.y - m1 * p3.x;
    double m2 = -1.0 / m1;
    double b2 = p2.y - (m2 * p2.x);

    double x = (b2 - b1) / (m1 - m2);
    double y = (b2 * m1 - b1 * m2) / (m1 - m2);

    double delta_x = x - p2.x;
    double delta_y = y - p2.y;

    pc_points[2].x = x;
    pc_points[2].y = y;
    pc_points[3].x = pc_points[0].x + delta_x;
    pc_points[3].y = pc_points[0].y + delta_y;
  }
}

void ClusterFilter::updateDimentionAndEstimatedAngle(
    std::vector<cv::Point2f> pc_points, autoware_msgs::CloudCluster &cluster) {

  cv::Point2f p1 = pc_points[0];
  cv::Point2f p2 = pc_points[1];
  cv::Point2f p3 = pc_points[2];

  double dist1 =
      sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  double dist2 =
      sqrt((p3.x - p2.x) * (p3.x - p2.x) + (p3.y - p2.y) * (p3.y - p2.y));
  double bb_yaw;
  // dist1 is length
  if (dist1 > dist2) {
    bb_yaw = atan2(p1.y - p2.y, p1.x - p2.x);
    cluster.bounding_box.dimensions.x = dist1;
    cluster.bounding_box.dimensions.y = dist2;
    cluster.bounding_box.dimensions.z = 2;
  } else {
    bb_yaw = atan2(p3.y - p2.y, p3.x - p2.x);
    cluster.bounding_box.dimensions.x = dist2;
    cluster.bounding_box.dimensions.y = dist1;
    cluster.bounding_box.dimensions.z = 2;
  }
  // convert yaw to quartenion
  tf::Matrix3x3 obs_mat;
  obs_mat.setEulerYPR(bb_yaw, 0, 0);

  tf::Quaternion q_tf;
  obs_mat.getRotation(q_tf);
  cluster.bounding_box.pose.orientation.x = q_tf.getX();
  cluster.bounding_box.pose.orientation.y = q_tf.getY();
  cluster.bounding_box.pose.orientation.z = q_tf.getZ();
  cluster.bounding_box.pose.orientation.w = q_tf.getW();
}

void ClusterFilter::getBBoxes(
    autoware_msgs::CloudClusterArray in_cluster_array,
    autoware_msgs::CloudClusterArray &out_cluster_array) {

  out_cluster_array.header = in_cluster_array.header;

  for (int i_cluster = 0; i_cluster < in_cluster_array.clusters.size();
       i_cluster++) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Convert from ros msg to PCL::PointCloud data type
    pcl::fromROSMsg(in_cluster_array.clusters[i_cluster].cloud, cloud);

    // calculating offset so that shape fitting would be visualized nicely
    cv::Mat m(pic_scale_ * roi_m_, pic_scale_ * roi_m_, CV_8UC1, cv::Scalar(0));
    float init_px = cloud[0].x + roi_m_ / 2;
    float init_py = cloud[0].y + roi_m_ / 2;
    int init_x = floor(init_px * pic_scale_);
    int init_y = floor(init_py * pic_scale_);
    int init_pic_x = init_x;
    int init_pic_y = pic_scale_ * roi_m_ - init_y;
    int offset_init_x = roi_m_ * pic_scale_ / 2 - init_pic_x;
    int offset_init_y = roi_m_ * pic_scale_ / 2 - init_pic_y;

    int num_points = cloud.size();
    std::vector<cv::Point> point_vec(num_points);
    std::vector<cv::Point2f> pc_points(4);
    float min_mx, min_my, max_mx, max_my;
    float min_m = 999;
    float max_m = -999;
    float max_z = -99;

    // for center of gravity
    // float sumX = 0; float sumY = 0;

    for (int i_point = 0; i_point < num_points; i_point++) {
      float p_x = cloud[i_point].x;
      float p_y = cloud[i_point].y;
      float p_z = cloud[i_point].z;
      // cast (roi_m_/2 < x,y < roi_m_/2) into (0 < x,y < roi_m_)
      float roi_x = p_x + roi_m_ / 2;
      float roi_y = p_y + roi_m_ / 2;
      // cast (roi_m_)mx(roi_m_)m into 900x900 scale
      int x = floor(roi_x * pic_scale_);
      int y = floor(roi_y * pic_scale_);
      // cast into image coordinate
      int pic_x = x;
      int pic_y = pic_scale_ * roi_m_ - y;
      // offset so that the object would be locate at the center
      int offset_x = pic_x + offset_init_x;
      int offset_y = pic_y + offset_init_y;
      m.at<uchar>(offset_y, offset_x) = 255;
      point_vec[i_point] = cv::Point(offset_x, offset_y);
      // calculate min and max slope for x1, x3(edge points)
      float m = p_y / p_x;
      if (m < min_m) {
        min_m = m;
        min_mx = p_x;
        min_my = p_y;
      }
      if (m > max_m) {
        max_m = m;
        max_mx = p_x;
        max_my = p_y;
      }

      // get maxZ
      if (p_z > max_z)
        max_z = p_z;

      // for center of gravity
      // sumX += offsetX;
      // sumY += offsetY;
    }

    // L shape fitting parameters
    float x_dist = max_mx - min_mx;
    float y_dist = max_my - min_my;
    float slope_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    float slope = (max_my - min_my) / (max_mx - min_mx);

    // random variable
    std::mt19937_64 mt;
    mt.seed(in_cluster_array.header.stamp.toSec());
    std::uniform_int_distribution<> rand_points(0, num_points - 1);

    // start l shape fitting for car like object
    // lSlopeDist_ = 2.0m
    if (slope_dist > slope_dist_ && num_points > num_points_) {
      float max_dist = 0;
      float max_dx, max_dy;

      // 80 random points, get max distance
      for (int i = 0; i < ram_points_; i++) {
        int p_ind = rand_points(mt);
        assert(p_ind >= 0 && p_ind < cloud.size());
        float x_i = cloud[p_ind].x;
        float y_i = cloud[p_ind].y;

        // from equation of distance between line and point
        float dist = abs(slope * x_i - 1 * y_i + max_my - slope * max_mx) /
                     sqrt(slope * slope + 1);
        if (dist > max_dist) {
          max_dist = dist;
          max_dx = x_i;
          max_dy = y_i;
        }
      }

      // for center of gravity
      // maxDx = sumX/cloud.size();
      // maxDy = sumY/cloud.size();

      // vector adding
      float max_m_vec_x = max_mx - max_dx;
      float max_m_vec_y = max_my - max_dy;
      float min_m_vec_x = min_mx - max_dx;
      float min_m_vec_y = min_my - max_dy;
      float last_x = max_dx + max_m_vec_x + min_m_vec_x;
      float last_y = max_dy + max_m_vec_y + min_m_vec_y;

      pc_points[0] = cv::Point2f(min_mx, min_my);
      pc_points[1] = cv::Point2f(max_dx, max_dy);
      pc_points[2] = cv::Point2f(max_dx, max_my);
      pc_points[3] = cv::Point2f(last_x, last_y);
      // bool is_promising = ruleBasedFilter(pc_points, max_z, num_points);
      // if (!is_promising)
      //   continue;
      // ------start visualization-----
      // cast (-15 < x,y < 15) into (0 < x,y < 30)
      //            float a = maxMx + roi_m_/2;
      //            float b = maxMy + roi_m_/2;
      //            float c = minMx + roi_m_/2;
      //            float d = minMy + roi_m_/2;
      //            float e = maxDx + roi_m_/2;
      //            float f = maxDy + roi_m_/2;
      //            float g = lastX + roi_m_/2;
      //            float h = lastY + roi_m_/2;
      //            // cast 30mx30m into 900x900 scale
      //            int aa = floor(a*pic_scale_);
      //            int bb = floor(b*pic_scale_);
      //            int cc = floor(c*pic_scale_);
      //            int dd = floor(d*pic_scale_);
      //            int ee = floor(e*pic_scale_);
      //            int ff = floor(f*pic_scale_);
      //            int gg = floor(g*pic_scale_);
      //            int hh = floor(h*pic_scale_);
      //            // cast into image coordinate
      //            int aaa = aa;
      //            int bbb = pic_scale_*roi_m_ - bb;
      //            int ccc = cc;
      //            int ddd = pic_scale_*roi_m_ - dd;
      //            int eee = ee;
      //            int fff = pic_scale_*roi_m_ - ff;
      //            int ggg = gg;
      //            int hhh = pic_scale_*roi_m_ - hh;
      //            // offset so that the object would be locate at the center
      //            int aaaa = aaa + offsetInitX;
      //            int bbbb = bbb + offsetInitY;
      //            int cccc = ccc + offsetInitX;
      //            int dddd = ddd + offsetInitY;
      //            int eeee = eee + offsetInitX;
      //            int ffff = fff + offsetInitY;
      //            int gggg = ggg + offsetInitX;
      //            int hhhh = hhh + offsetInitY;
      //
      //            line( m, Point(aaaa, bbbb), Point(cccc, dddd),
      //            Scalar(255,255,0), 1, 8 );
      //            line( m, Point(aaaa, bbbb), Point(eeee, ffff),
      //            Scalar(255,255,0), 1, 8 );
      //            line( m, Point(cccc, dddd), Point(eeee, ffff),
      //            Scalar(255,255,0), 1, 8 );
      //            line( m, Point(aaaa, bbbb), Point(gggg, hhhh),
      //            Scalar(255,255,0), 1, 8 );
      //            line( m, Point(cccc, dddd), Point(gggg, hhhh),
      //            Scalar(255,255,0), 1, 8 );
      //
      //            imshow("Display Image", m);
      //            waitKey(0);
      // --------end visualization -----------

    } else {
      // MAR fitting
      cv::RotatedRect rect_info = cv::minAreaRect(point_vec);
      cv::Point2f rect_points[4];
      rect_info.points(rect_points);
      // covert points back to lidar coordinate
      getPointsInPcFrame(rect_points, pc_points, offset_init_x, offset_init_y);
      // rule based filter
      // bool is_promising = ruleBasedFilter(pc_points, max_z, num_points);
      // if (!is_promising)
      //   continue;
      // for visualization
      //            for( int j = 0; j < 4; j++ )
      //                line( m, rectPoints[j], rectPoints[(j+1)%4],
      //                Scalar(255,255,0), 1, 8 );
      //            imshow("Display Image", m);
      //            waitKey(0);
    }

    updateCpFromPoints(pc_points, in_cluster_array.clusters[i_cluster]);
    // std::cout << iCluster << "th cx
    // "<<inClusterArray.clusters[iCluster].centroid_point.point.x << std::endl;
    // update pcPoints to make it right angle bbox
    toRightAngleBBox(pc_points);

    updateDimentionAndEstimatedAngle(pc_points,
                                     in_cluster_array.clusters[i_cluster]);

    out_cluster_array.clusters.push_back(in_cluster_array.clusters[i_cluster]);
    std::cout
        << "x: "
        << in_cluster_array.clusters[i_cluster].bounding_box.pose.position.x
        << std::endl;
    std::cout
        << "y: "
        << in_cluster_array.clusters[i_cluster].bounding_box.pose.position.y
        << std::endl;
  }
  // cout << clusterArray.clusters[2].centroid_point.point.x<<endl;
}
