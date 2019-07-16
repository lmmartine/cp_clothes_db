/*
 * CPCLOTHESDB.h
 *
 *  Created on: 01-2018
 *      Author: luz.martinez@amtc.cl
 */

#ifndef CPCLOTHESDB_H_
#define CPCLOTHESDB_H_

// C, C++
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <list>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
 #include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


// pcl
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>


// srvs and msgs
#include <cp_clothes_db/Onoff.h>
#include <geometry_msgs/Pose.h>

namespace uchile_perception {

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;


using std::string;

class CPCLOTHESDB  {


private:
	 ros::NodeHandle priv;
	std::string _name;
	bool _enabled;

	// - - - - - Parameters - - - - - - -
	std::string _base_frame;
	std::string _rgbd_frame;
	std::string _rgb_frame;
	std::string _depth_frame;
	std::string _rgb_topic;
	std::string _depth_topic;
	std::string _point_topic;
	std::string _imagePoint_topic;

	std::string _class;
	int _idmove;
	int _idimg;
	int nxtion;
	std::string _gripper_frame;
	float last_gripper_position;
	bool _cut_robot;
	float _crop_width;
	float _crop_depth;
	float _crop_min_z;
	float _crop_max_z;

	ros::Time lasttime_transform;


	// publishers
	image_transport::Publisher _mask_pub;

	//Subscribers 
	ros::Subscriber _subs_rgb;
	ros::Subscriber _subs_depth;
	ros::Subscriber _subs_point;

	pcl::PCLPointCloud2::ConstPtr cloud_in;
	sensor_msgs::ImageConstPtr rgb_in;
	sensor_msgs::ImageConstPtr depth_in;
	std::string image_frameid;


	// services
    ros::ServiceServer _active_server;

	// Listeners
	ros::Subscriber _depth_sub;
	tf::TransformListener *_tf_listener;
	tf::TransformListener tf_listener;
	// pcl
	pcl::VoxelGrid<Point> sor;


public:
	bool _is_on;
	bool ready_rgb;
	bool ready_depth;
	bool ready_point;

	CPCLOTHESDB(std::string name);
	virtual ~CPCLOTHESDB();

	void run();


private:

	// - - - - - - S u b s c r i b e r   C a l l b a c k s  - - - - - - - - -
	void _process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in);
	void _process_depth(const sensor_msgs::ImageConstPtr& img);
	void _process_rgb(const sensor_msgs::ImageConstPtr& img);

	// - - - - - - - - - - - - S e r v i c e s - - - - - - - - -
	bool _active_service(cp_clothes_db::Onoff::Request  &req, cp_clothes_db::Onoff::Response &res) ;


	// - - - - -  Functions  - - - - - - - - - -
	void save_images (cv::Mat rgbimg, cv::Mat  depthimg, cv::Mat mask);
	geometry_msgs::Pose get_gripper_position ();
	void downsample_pointcloud(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);
	bool transform_pointcloud(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, std::string target_frame);
	void crop_pointcloud( PointCloud cloud_in, PointCloud::Ptr cloud_out);
	void crop_robot_pointcloud(PointCloud cloud_in, PointCloud::Ptr cloud_out, float gripper_position);
};

} /* namespace  */
#endif /* CPCLOTHESDB_H_ */



