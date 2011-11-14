/*
// Authors: Gabriele Fanelli, Thibaut Weise, Juergen Gall, BIWI, ETH Zurich
// Email: fanelli@vision.ee.ethz.ch

// You may use, copy, reproduce, and distribute this Software for any
// non-commercial purpose, subject to the restrictions of the
// Microsoft Research Shared Source license agreement ("MSR-SSLA").
// Some purposes which can be non-commercial are teaching, academic
// research, public demonstrations and personal experimentation. You
// may also distribute this Software with books or other teaching
// materials, or publish the Software on websites, that are intended
// to teach the use of the Software for academic or other
// non-commercial purposes.
// You may not use or distribute this Software or any derivative works
// in any form for commercial purposes. Examples of commercial
// purposes would be running business operations, licensing, leasing,
// or selling the Software, distributing the Software for use with
// commercial products, using the Software in the creation or use of
// commercial products or any other activity which purpose is to
// procure a commercial gain to you or others.
// If the Software includes source code or data, you may create
// derivative works of such portions of the Software and distribute
// the modified Software for non-commercial purposes, as provided
// herein.

// THE SOFTWARE COMES "AS IS", WITH NO WARRANTIES. THIS MEANS NO
// EXPRESS, IMPLIED OR STATUTORY WARRANTY, INCLUDING WITHOUT
// LIMITATION, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
// PARTICULAR PURPOSE, ANY WARRANTY AGAINST INTERFERENCE WITH YOUR
// ENJOYMENT OF THE SOFTWARE OR ANY WARRANTY OF TITLE OR
// NON-INFRINGEMENT. THERE IS NO WARRANTY THAT THIS SOFTWARE WILL
// FULFILL ANY OF YOUR PARTICULAR PURPOSES OR NEEDS. ALSO, YOU MUST
// PASS THIS DISCLAIMER ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR
// DERIVATIVE WORKS.

// NEITHER MICROSOFT NOR ANY CONTRIBUTOR TO THE SOFTWARE WILL BE
// LIABLE FOR ANY DAMAGES RELATED TO THE SOFTWARE OR THIS MSR-SSLA,
// INCLUDING DIRECT, INDIRECT, SPECIAL, CONSEQUENTIAL OR INCIDENTAL
// DAMAGES, TO THE MAXIMUM EXTENT THE LAW PERMITS, NO MATTER WHAT
// LEGAL THEORY IT IS BASED ON. ALSO, YOU MUST PASS THIS LIMITATION OF
// LIABILITY ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR DERIVATIVE
// WORKS.

// When using this software, please acknowledge the effort that
// went into development by referencing the paper:
//
// Fanelli G., Weise T., Gall J., Van Gool L., Real Time Head Pose Estimation from Consumer Depth Cameras
// 33rd Annual Symposium of the German Association for Pattern Recognition (DAGM'11), 2011

*/

#include <string>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include "CRForestEstimator.h"

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <kdl/frames.hpp>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
using namespace cv;
using namespace angles;

// Path to trees
string g_treepath;
// Number of trees
int g_ntrees;
// Patch width
int g_p_width;
// Patch height
int g_p_height;
//maximum distance form the sensor - used to segment the person
int g_max_z = 0;
//head threshold - to classify a cluster of votes as a head
int g_th = 400;
//threshold for the probability of a patch to belong to a head
float g_prob_th = 1.0f;
//threshold on the variance of the leaves
double g_maxv = 800.f;
//stride (how densely to sample test patches - increase for higher speed)
int g_stride = 5;
//radius used for clustering votes into possible heads
double g_larger_radius_ratio = 1.f;
//radius used for mean shift
double g_smaller_radius_ratio = 6.f;

//pointer to the actual estimator
CRForestEstimator* g_Estimate;
//input 3D image
Mat g_im3D;

CRForestEstimator estimator;
ros::Publisher cloud_pub;
ros::Publisher pose_pub;

std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
std::vector< std::vector< Vote > > g_clusters; //full clusters of votes
std::vector< Vote > g_votes; //all votes returned by the forest

cv::Rect roi;
bool roiReady = false;

void loadConfig(ros::NodeHandle nh) {
	nh.param("tree_path",			g_treepath,				string("trees/tree"));
	nh.param("ntrees",				g_ntrees,				10);
	nh.param("max_variance",		g_maxv,					800.0);
	nh.param("larger_radius_ratio",	g_larger_radius_ratio,	1.0);
	nh.param("smaller_radius_ratio",g_smaller_radius_ratio,	6.0);
	nh.param("stride",				g_stride,				5);
	nh.param("head_threshold",		g_th,					400);	
}

void roiCallback(const sensor_msgs::RegionOfInterest::ConstPtr& msg) {
	roi = cv::Rect(msg->x_offset, msg->y_offset, msg->width, msg->height);
	roiReady = true;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

	PointCloud cloud;
	pcl::fromROSMsg(*msg, cloud);

	Mat img3D;
	img3D.create(cloud.height, cloud.width, CV_32FC3);
	
	int yMin, xMin, yMax, xMax;
	yMin = 0; xMin = 0;
	yMax = img3D.rows; xMax = img3D.cols;
	if(roiReady) {
		yMin = roi.y;
		xMin = roi.x;
		yMax = yMin + roi.height;
		xMax = xMin + roi.width;
	}
	//ROS_INFO("x=%d, y=%d", x, y);
	//get 3D from depth
	for(int y = yMin ; y < img3D.rows; y++) {
		Vec3f* img3Di = img3D.ptr<Vec3f>(y);
	
		for(int x = xMin; x < img3D.cols; x++) {
			if(cloud.at(x,y).z < 2) { //this part is a bit of a hack - eventaully, do real head segmentation
				img3Di[x][0] = cloud.at(x, y).x*1000;
				img3Di[x][1] = cloud.at(x, y).y*1000;
				img3Di[x][2] = cloud.at(x, y).z*1000;
			}
		}
	}

	g_means.clear();
	g_votes.clear();
	g_clusters.clear();
	
	//do the actual estimate
	estimator.estimate( 	img3D,
							g_means,
							g_clusters,
							g_votes,
							g_stride,
							g_maxv,
							g_prob_th,
							g_larger_radius_ratio,
							g_smaller_radius_ratio,
							false,
							g_th
						);
	
	cout << "Heads found : " << g_means.size() << endl;
	
	//assuming there's only one head in the image!
	if(g_means.size()>0){
	
		cout << "Estimated: " << g_means[0][0] << " " << g_means[0][1] << " " << g_means[0][2] << " " << g_means[0][3] << " " << g_means[0][4] << " " << g_means[0][5] <<endl;
	
		float pt2d_est[2];
		float pt2d_gt[2];	
	}
	PointCloud::Ptr out (new PointCloud);
	out->header.stamp = ros::Time::now();
	out->header.frame_id = msg->header.frame_id;
	
	cv::Vec<float,POSE_SIZE> pose(g_means[0]);
	
	
	out->points.push_back(pcl::PointXYZ(pose[0]/1000, pose[1]/1000, pose[2]/1000));
	
	KDL::Rotation r = KDL::Rotation::RPY(
										 from_degrees(pose[4]), 
										 from_degrees(pose[3]+90), 
										 from_degrees(pose[5])
										); //don't forget to convert these to radians
	double qx, qy, qz, qw;
	r.GetQuaternion(qx, qy, qz, qw);
	
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header = out->header;
	pose_msg.pose.position.x = out->points[0].x;
	pose_msg.pose.position.y = out->points[0].y;
	pose_msg.pose.position.z = out->points[0].z;
	
	pose_msg.pose.orientation.x = qx;
	pose_msg.pose.orientation.y = qy;
	pose_msg.pose.orientation.z = qz;
	pose_msg.pose.orientation.w = qw;
	
	cloud_pub.publish(out);
	pose_pub.publish(pose_msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "head_pose_estimator");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback);
	ros::Subscriber roi_sub = nh.subscribe<sensor_msgs::RegionOfInterest>("roi", 1, roiCallback);
	
	cloud_pub = nh.advertise<PointCloud>("normal", 1);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("head_pose", 1);
	
	loadConfig(nh);
	ROS_INFO("tree path: %s", g_treepath.c_str());
	if( !estimator.loadForest(g_treepath.c_str(), g_ntrees) ){
		ROS_ERROR("could not read forest!");
		exit(-1);
	}

	ros::spin();
	return 0;

}
