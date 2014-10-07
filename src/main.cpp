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

#include <stdlib.h>
#include <string>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include "head_pose_estimation/CRForestEstimator.h"

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
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "people_msgs/PositionMeasurement.h"
#include "people_msgs/PositionMeasurementArray.h"

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

// Frame to transform the head pose to
string g_head_target_frame;

bool g_head_depth_ready = false;
bool g_cloud_ready = false;
bool g_transform_ready = false;

double g_head_depth = 0.5;

string g_cloud_frame;

double g_roll_bias = 0;
double g_pitch_bias = 0;
double g_yaw_bias = 0;

CRForestEstimator estimator;
ros::Publisher pose_pub;

tf::TransformListener* listener;
tf::TransformBroadcaster* broadcaster;
tf::StampedTransform g_transform;

std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
std::vector< std::vector< Vote > > g_clusters; //full clusters of votes
std::vector< Vote > g_votes; //all votes returned by the forest

void loadConfig() {
	ros::NodeHandle nh("~");
	nh.param("tree_path",           g_treepath,             string("trees/tree"));
	nh.param("ntrees",              g_ntrees,               10);
	nh.param("max_variance",        g_maxv,                 800.0);
	nh.param("larger_radius_ratio", g_larger_radius_ratio,  1.0);
	nh.param("smaller_radius_ratio",g_smaller_radius_ratio, 6.0);
	nh.param("stride",              g_stride,               5);
	nh.param("head_threshold",      g_th,                   400);
	nh.param("head_target_frame",   g_head_target_frame,    string("/camera_depth_frame"));
	nh.param("angular_bias_roll",   g_roll_bias,            0.0);
	nh.param("angular_bias_pitch",  g_pitch_bias,           0.0);
	nh.param("angular_bias_yaw",    g_yaw_bias,             0.0);

	
}

void peopleCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg) {
	if(g_cloud_ready && (msg->people.size() > 0)) {
		g_head_depth_ready = true;
		people_msgs::PositionMeasurement person = msg->people[0];
		geometry_msgs::PointStamped head_point, head_point_transformed;
		head_point.header = person.header;
		head_point.point = person.pos;
		listener->transformPoint(g_cloud_frame, head_point, head_point_transformed);
		g_head_depth = head_point_transformed.point.z;
	}
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	PointCloud cloud;
	pcl::fromROSMsg(*msg, cloud);

	g_cloud_frame = cloud.header.frame_id;
	g_cloud_ready = true;

	if(!g_head_depth_ready) return;

	Mat img3D;
	img3D = Mat::zeros(cloud.height, cloud.width, CV_32FC3);
	//img3D.create(cloud.height, cloud.width, CV_32FC3);
	
	int yMin, xMin, yMax, xMax;
	yMin = 0; xMin = 0;
	yMax = img3D.rows; xMax = img3D.cols;

	//get 3D from depth
	for(int y = yMin ; y < img3D.rows; y++) {
		Vec3f* img3Di = img3D.ptr<Vec3f>(y);
	
		for(int x = xMin; x < img3D.cols; x++) {
			pcl::PointXYZ p = cloud.at(x,y);
			if((p.z>g_head_depth-0.2) && (p.z<g_head_depth+0.2)) {
				img3Di[x][0] = p.x*1000;
				img3Di[x][1] = p.y*1000;
				img3Di[x][2] = hypot(img3Di[x][0], p.z*1000); //they seem to have trained with incorrectly projected 3D points
				//img3Di[x][2] = p.z*1000;
			} else {
				img3Di[x] = 0;
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
	
	//assuming there's only one head in the image!
	if(g_means.size() > 0) {	
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.header.frame_id = msg->header.frame_id;
	
		cv::Vec<float,POSE_SIZE> pose(g_means[0]);

		tf::Quaternion q;
		q.setRPY(
			from_degrees( pose[5]+180+g_roll_bias ), 
			from_degrees(-pose[3]+180+g_pitch_bias),
			from_degrees(-pose[4]+    g_yaw_bias  )
		);
	
		geometry_msgs::PointStamped head_point;
		geometry_msgs::PointStamped head_point_transformed;
	
		head_point.header = pose_msg.header;

		head_point.point.x = pose[0]/1000;
		head_point.point.y = pose[1]/1000;
		head_point.point.z = pose[2]/1000;
	
		try {
			listener->waitForTransform(head_point.header.frame_id, g_head_target_frame, ros::Time::now(), ros::Duration(2.0));
			listener->transformPoint(g_head_target_frame, head_point, head_point_transformed);
		} catch(tf::TransformException ex) {
			ROS_WARN(
				"Transform exception, when transforming point from %s to %s\ndropping pose (don't worry, this is probably ok)",
				head_point.header.frame_id.c_str(), g_head_target_frame.c_str());
				ROS_WARN("Exception was %s", ex.what());
			return;
		}
		
		tf::Transform trans;
		
		pose_msg.pose.position = head_point_transformed.point;
		pose_msg.header.frame_id = head_point_transformed.header.frame_id;
	
		pose_msg.pose.orientation.x = q.x();
		pose_msg.pose.orientation.y = q.y();
		pose_msg.pose.orientation.z = q.z();
		pose_msg.pose.orientation.w = q.w();

		trans.setOrigin(tf::Vector3(
			pose_msg.pose.position.x, 
			pose_msg.pose.position.y, 
			pose_msg.pose.position.z
		));
		trans.setRotation(q);
		g_transform = tf::StampedTransform(trans, pose_msg.header.stamp, pose_msg.header.frame_id, "head_origin");
		// broadcaster->sendTransform(tf::StampedTransform(trans, pose_msg.header.stamp, pose_msg.header.frame_id, "head_origin"));
		g_transform_ready = true;
		pose_msg.header.stamp = ros::Time::now();
		geometry_msgs::PoseStamped zero_pose;
		zero_pose.header.frame_id = "head_origin";
		zero_pose.header.stamp = ros::Time::now();
		zero_pose.pose.orientation.w = 1;
		//pose_pub.publish(pose_msg);
		pose_pub.publish(zero_pose);
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "head_pose_estimator");
	ros::NodeHandle nh;
	
	listener = new tf::TransformListener();
	broadcaster = new tf::TransformBroadcaster();
	
	image_transport::ImageTransport it(nh);
	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback);
	ros::Subscriber face_pos_sub = nh.subscribe<people_msgs::PositionMeasurementArray>("/face_detector/people_tracker_measurements_array", 1, peopleCallback);
	
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("head_pose", 1);
	
	loadConfig();
	ROS_INFO("tree path: %s", g_treepath.c_str());
	if( !estimator.loadForest(g_treepath.c_str(), g_ntrees) ){
		ROS_ERROR("could not read forest!");
		exit(-1);
	}
	
	ros::Rate rate(20);
	while(ros::ok()) {
		if(g_transform_ready) {
			g_transform.stamp_ = ros::Time::now() + ros::Duration(1.0);
			broadcaster->sendTransform(g_transform);
		}
		ros::spinOnce();
		rate.sleep();
	}	
	return 0;
}
