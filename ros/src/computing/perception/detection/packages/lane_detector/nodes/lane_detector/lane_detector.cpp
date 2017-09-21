/*
 *  Copyright (c) 2015, Nagoya University
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
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <list>
#include "utils.h"
#include <string>

#include <autoware_msgs/ImageLaneObjects.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

static ros::Publisher image_lane_objects;
static ros::Publisher image_with_lines;

std::vector<CvPoint> g_old_left_lane;
std::vector<CvPoint> g_old_right_lane;

#define GREEN  CV_RGB(0, 255, 0)
#define RED    CV_RGB(255, 0, 0)
#define BLUE   CV_RGB(0, 0, 255)
#define PURPLE CV_RGB(255, 0, 255)
#define CLIP_RATIO 	1.75
#define HOUGH_RHO  	1
#define HOUGH_THETA CV_PI/180

enum {
	SCAN_STEP           	= 5,      // in pixels
	LINE_REJECT_DEGREES 	= 20,     // in degrees
	BW_TRESHOLD         	= 250,    // edge response strength to recognize for 'WHITE'
	BORDERX             	= 10,     // px, skip this much from left & right borders
	MAX_RESPONSE_DIST   	= 5,      // px

	CANNY_MIN_TRESHOLD 		= 1,       // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD 		= 100,     // edge detector maximum hysteresis threshold

	HOUGH_TRESHOLD        	= 50,   // line approval vote threshold
	HOUGH_MIN_LINE_LENGTH 	= 50,   // remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP    	= 50   // join lines to one with smaller than this gaps
};

static void FilterLinesAndAverage(std::vector<std::pair<CvPoint*, double> >& lines, double min_d, std::pair<CvPoint, CvPoint>& avg_line)
{
	if(lines.size() == 0) return;

	double min_y = 10000, max_y = 0, min_x=0, max_x=0;

	for(int i = 0 ; i < lines.size(); i++)
	{
		if(lines.at(i).second > min_d + 15)
		{
			lines.erase(lines.begin()+i);
			i--;
		}
	}

	for(int i = 0 ; i < lines.size(); i++)
	{
		if(lines.at(i).first[0].y >= max_y)
		{
			max_y = lines.at(i).first[0].y;
			min_x = lines.at(i).first[0].x;
		}

		if(lines.at(i).first[1].y >= max_y)
		{
			max_y = lines.at(i).first[1].y;
			min_x = lines.at(i).first[1].x;
		}

		if(lines.at(i).first[0].y <= min_y)
		{
			min_y = lines.at(i).first[0].y;
			max_x = lines.at(i).first[0].x;
		}

		if(lines.at(i).first[1].y <= min_y)
		{
			min_y = lines.at(i).first[1].y;
			max_x = lines.at(i).first[1].x;
		}
	}

	avg_line.first.x = max_x;
	avg_line.first.y = min_y;

	avg_line.second.x = min_x;
	avg_line.second.y = max_y;
}

static bool LineIntersect(CvPoint& l1p1, CvPoint& l1p2, CvPoint& l2p1, CvPoint& l2p2, CvPoint& inter_point)
{
	CvPoint x = cvPoint(l2p1.x-l1p1.x,  l2p1.y-l1p1.y);
	CvPoint d1 =cvPoint(l1p2.x-l1p1.x, l1p2.y-l1p1.y);
	CvPoint d2 =cvPoint(l2p2.x-l2p1.x, l2p2.y-l2p1.y);

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (fabs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    inter_point = cvPoint(l1p1.x + d1.x * t1, l1p1.y + d1.y * t1);
    return true;
}

static void ApplyMask(IplImage *img)
{
	int nPoints = 4;
	int vanish_point = img->width/2.0;
	CvPoint* center = new CvPoint[nPoints];
	CvPoint* left_corner = new CvPoint[3];
	CvPoint* right_corner = new CvPoint[3];

	center[0].x = 50;
	center[0].y = img->height;

	center[1].x = img->width-50;
	center[1].y = img->height;

	center[2].x = vanish_point;
	center[2].y = 100;

	center[3].x = vanish_point;
	center[3].y = 100;

	left_corner[0].x = 0;left_corner[0].y = 0;
	left_corner[1].x = 0;left_corner[1].y = img->height-img->height/4;
	left_corner[2].x = vanish_point-200;
	if(left_corner[2].x< 0)
		left_corner[2].x = 0;
	left_corner[2].y = 0;

	right_corner[0].x = vanish_point+200;
	if(right_corner[0].x > img->width)
		right_corner[0].x = img->width;
	right_corner[0].y = 0;
	right_corner[1].x = img->width;right_corner[1].y = img->height-img->height/4;
	right_corner[2].x = img->width;right_corner[2].y = 0;

	IplImage *mask = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	IplImage *filtered_img = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);

	cvSet(mask, cvScalar(255));
	cvFillConvexPoly( mask, center, nPoints, cvScalar(0));
	cvFillConvexPoly( mask, left_corner, 3, cvScalar(0));
	cvFillConvexPoly( mask, right_corner, 3, cvScalar(0));

	cvAnd(img, mask, filtered_img);
	cvCopy(filtered_img, img);

	/**
	 * For Mask Visualization
	 */
//	cvShowImage("Mask", mask);
//	cvMoveWindow("Mask", 0, 3*(img->height+25));
//	cvReleaseImage(&mask);
//
//	cvShowImage("Filtered", filtered_img);
//	cvMoveWindow("Filtered", 0, 4*(img->height+25));
//	cvReleaseImage(&filtered_img);
}

static void ExtractLeftAndRightLanes(CvSize& frame_size, CvSeq *lines, double& min_right, double& min_left, std::vector<std::pair<CvPoint*, double> >& left_lines, std::vector<std::pair<CvPoint*, double> >& right_lines)
{
	for (int i=0; i<lines->total; i++)
	{
		CvPoint *line = (CvPoint *)cvGetSeqElem(lines, i);
		int dx = line[1].x - line[0].x;
		int dy = line[1].y - line[0].y;
		double angle = atan2(dy, dx) * 180/CV_PI;

		if (fabs(angle) <= LINE_REJECT_DEGREES || fabs(angle) >= 180 - LINE_REJECT_DEGREES) // reject near horizontal lines
			continue;

		CvPoint van_low = cvPoint(frame_size.width/2.0, frame_size.height);
		if(angle > 0)
		{
			//cvLine(edges, cvPoint(line[0].x, line[0].y), cvPoint(line[1].x, line[1].y), CV_RGB(255, 0, 0), 1);
			CvPoint inter_p = cvPoint(van_low.x+75, van_low.y-50);
			CvPoint perp_p;
			bool bFound = LineIntersect(van_low, inter_p, line[0], line[1], perp_p);
			if(bFound)
			{
				//cvLine(original_croped, van_low, perp_p, CV_RGB(255, 255, 0), 4);
				double d = hypot(perp_p.x - van_low.x, perp_p.y-van_low.y);
				if(d < min_right)
					min_right = d;
				right_lines.push_back(std::make_pair(line, d));
			}
		}
		else
		{
			//cvLine(edges, cvPoint(line[0].x, line[0].y), cvPoint(line[1].x, line[1].y), CV_RGB(0, 255, 0), 1);
			CvPoint inter_p = cvPoint(van_low.x-75, van_low.y-50);
			CvPoint perp_p;
			bool bFound = LineIntersect(van_low, inter_p, line[0], line[1], perp_p);
			if(bFound)
			{
				//cvLine(original_croped, van_low, perp_p, CV_RGB(0, 255, 255), 4);
				double d = hypot(perp_p.x - van_low.x, perp_p.y-van_low.y);
				if(d < min_left)
					min_left = d;
				left_lines.push_back(std::make_pair(line, d));
			}
		}
	}
}

static void process_image_common(IplImage *frame)
{
	CvSize video_size 			= cvSize(frame->width, frame->height);
	CvSize    frame_size 		= cvSize(video_size.width, (double)video_size.height/CLIP_RATIO);
	CvRect    rect 				= cvRect(0, frame->height - frame_size.height, frame_size.width, frame_size.height);
	IplImage *original_croped 	= cvCreateImage(frame_size, IPL_DEPTH_8U, 3);
	IplImage *gray       		= cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *edges      		= cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	CvMemStorage *houghStorage 	= cvCreateMemStorage(0);

	/**
	 * 1.
	 * Clip the Image, Extract bottom part with road information
	 */
	cvSetImageROI(frame, rect);
	cvCopy(frame, original_croped);
	cvResetImageROI(frame);
	cvCvtColor(original_croped, gray, CV_BGR2GRAY);

	/**
	 * 2.
	 * Smooth the Image and apply canny edge detection
	 */
	cvSmooth(gray, gray, CV_GAUSSIAN, 15, 15);
	cvCanny(gray, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);

	/**
	 * Apply bitwise Mask to remove unwanted lines from the canny step
	 */
	ApplyMask(edges);

	/**
	 * 3.
	 * Extract lines using Hough line detector
	 */
	CvSeq *lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC, HOUGH_RHO, HOUGH_THETA, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);

	std::vector<std::pair<CvPoint*, double> > left_lines;
	std::vector<std::pair<CvPoint*, double> > right_lines;
	double min_right = 10000;
	double min_left = 10000;

	/**
	 * 4.
	 *  Find the closest left and right lines to the center of the image
	 */
	ExtractLeftAndRightLanes(frame_size, lines, min_right, min_left, left_lines, right_lines);

	std::pair<CvPoint, CvPoint> right_line_avg;
	std::pair<CvPoint, CvPoint> left_line_avg;

	/**
	 * 5.
	 * Maximize and calculate the average for left and right lines, Extract initial left and right
	 */
	FilterLinesAndAverage(right_lines, min_right, right_line_avg);
	FilterLinesAndAverage(left_lines, min_left, left_line_avg);


	/**
	 * 6.
	 * Calculate initially extracted lines length
	 */
	double right_length = hypot(right_line_avg.first.x - right_line_avg.second.x, right_line_avg.first.y - right_line_avg.second.y);
	double left_length = hypot(left_line_avg.first.x - left_line_avg.second.x, left_line_avg.first.y - left_line_avg.second.y);
	CvPoint right_center;
	CvPoint left_center;

	/**
	 * 7.
	 * Extend Lines to start from the bottom to the top of the image
	 */
	CvPoint bottom_p1 = cvPoint(0,frame_size.height);
	CvPoint bottom_p2 = cvPoint(frame_size.width,frame_size.height);

	CvPoint center_p1 = cvPoint(0,7*frame_size.height/16);
	CvPoint center_p2 = cvPoint(frame_size.width,7*frame_size.height/16);

	CvPoint top_p1 = cvPoint(0,frame_size.height/8);
	CvPoint top_p2 = cvPoint(frame_size.width,frame_size.height/8);

	CvPoint inter_p;

	if(LineIntersect(center_p1, center_p2, right_line_avg.first, right_line_avg.second, inter_p))
		right_center = inter_p;

	if(LineIntersect(center_p1, center_p2, left_line_avg.first, left_line_avg.second, inter_p))
		left_center = inter_p;

	if(LineIntersect(bottom_p1, bottom_p2, right_line_avg.first, right_line_avg.second, inter_p))
		right_line_avg.first = inter_p;
	if(LineIntersect(top_p1, top_p2, right_line_avg.first, right_line_avg.second, inter_p))
		right_line_avg.second = inter_p;

	if(LineIntersect(bottom_p1, bottom_p2, left_line_avg.first, left_line_avg.second, inter_p))
		left_line_avg.first = inter_p;
	if(LineIntersect(top_p1, top_p2, left_line_avg.first, left_line_avg.second, inter_p))
		left_line_avg.second = inter_p;


	double top_length = hypot(right_line_avg.first.x - left_line_avg.first.x, right_line_avg.first.y - left_line_avg.first.y);

	double center_length = hypot(right_center.x - left_center.x, right_center.y - left_center.y);

	double bottom_length = hypot(right_line_avg.second.x - left_line_avg.second.x, right_line_avg.second.y - left_line_avg.second.y);

	//std::cout << "TopL: " << top_length << ", CenterL: " << center_length << ", BottomL: " << bottom_length << std::endl;

	/**
	 * 8.
	 * Check for lines and points consistency, skip noisy detection , and use previous lines in case of miss detection
	 */
	if(left_center.x < right_center.x && right_length > 20 && left_length > 20 && bottom_length > 25 && bottom_length < top_length &&
			left_line_avg.first.x < right_line_avg.first.x && left_line_avg.second.x < right_line_avg.second.x) // very good lane
	{
		//std::cout << "Perfect Lanes found !! .." << std::endl;
		if(g_old_left_lane.size() != 3)
		{
			g_old_left_lane.clear();
			g_old_left_lane.push_back(left_line_avg.first);
			g_old_left_lane.push_back(left_center);
			g_old_left_lane.push_back(left_line_avg.second);
		}
		else
		{
			g_old_left_lane.at(0) = left_line_avg.first;
			g_old_left_lane.at(1) = left_center;
			g_old_left_lane.at(2) = left_line_avg.second;
		}

		if(g_old_right_lane.size() != 3)
		{
			g_old_right_lane.clear();
			g_old_right_lane.push_back(right_line_avg.first);
			g_old_right_lane.push_back(right_center);
			g_old_right_lane.push_back(right_line_avg.second);
		}
		else
		{
			g_old_right_lane.at(0) = right_line_avg.first;
			g_old_right_lane.at(1) = right_center;
			g_old_right_lane.at(2) = right_line_avg.second;
		}
	}
	else
	{
		if(g_old_left_lane.size() == 3 && g_old_right_lane.size() == 3)
		{
			//std::cout << "Use old Lanes !! .." << std::endl;
			left_line_avg.first 	= g_old_left_lane.at(0);
			left_center 			= g_old_left_lane.at(1) ;
			left_line_avg.second 	= g_old_left_lane.at(2) ;

			right_line_avg.first 	= g_old_right_lane.at(0);
			right_center 			= g_old_right_lane.at(1);
			right_line_avg.second 	= g_old_right_lane.at(2);
		}
		else
		{
			//std::cout << "Nothig Good  !! .." << std::endl;
			left_line_avg.first 	= cvPoint(0,0);
			left_center 			= cvPoint(0,0);
			left_line_avg.second 	= cvPoint(0,0);
			right_line_avg.first 	= cvPoint(0,0);
			right_center 			= cvPoint(0,0);
			right_line_avg.second 	= cvPoint(0,0);
		}
	}

	cvLine(original_croped, right_line_avg.first, right_line_avg.second, CV_RGB(250, 100, 111), 5);
	cvLine(original_croped, left_line_avg.first, left_line_avg.second, CV_RGB(250, 100, 111), 5);

//	cvLine(original_croped, right_line_avg.first, left_line_avg.first, CV_RGB(0, 0, 255), 3);
//	cvLine(original_croped, right_center, left_center, CV_RGB(0, 0, 255), 3);
//	cvLine(original_croped, right_line_avg.second, left_line_avg.second, CV_RGB(0, 0, 255), 3);

	cvShowImage("Gray", gray);
	cvShowImage("Edges", edges);
	cvShowImage("Color", original_croped);

	cvMoveWindow("Gray", 0, 0);
	cvMoveWindow("Edges", 0, frame_size.height+25);
	cvMoveWindow("Color", 0, 2*(frame_size.height+25));

	cvReleaseMemStorage(&houghStorage);
	cvReleaseImage(&gray);
	cvReleaseImage(&edges);
	cvReleaseImage(&original_croped);

}

static void lane_cannyhough_callback(const sensor_msgs::Image& image_source)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
  IplImage frame = cv_image->image;
  process_image_common(&frame);
  cvWaitKey(2);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "line_ocv");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	std::string image_topic_name;
	private_nh.param<std::string>("image_raw_topic", image_topic_name, "/image_raw");
	ROS_INFO("Setting image topic to %s", image_topic_name.c_str());

	ros::Subscriber subscriber = n.subscribe(image_topic_name, 1, lane_cannyhough_callback);

	image_lane_objects = n.advertise<autoware_msgs::ImageLaneObjects>("lane_pos_xy", 1);
	image_with_lines = n.advertise<sensor_msgs::Image>("detected_lines", 1);

	ros::spin();

	return 0;
}
