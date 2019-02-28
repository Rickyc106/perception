#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "perception/crop.h"
#include "perception/segmentation.h"
#include "perception/SegmentationConfig.h"
#include "dynamic_reconfigure/server.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "Point Cloud Demo");

	ros::NodeHandle nh;

	ros::Publisher table_pub = nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visual_marker", 1);
	ros::Publisher object_pub = nh.advertise<sensor_msgs::PointCloud2>("clustered_objects", 1);

	perception::Segmenter segmenter(table_pub, marker_pub, object_pub);
	
	ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1);
	perception::Cropper cropper(crop_pub);

	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, 
					&perception::Segmenter::Callback, &segmenter);


	dynamic_reconfigure::Server<perception::SegmentationConfig> seg_server;
	dynamic_reconfigure::Server<perception::SegmentationConfig>::CallbackType seg_f;

	//dynamic_reconfigure::Server<perception::CropConfig> crop_server;
	//dynamic_reconfigure::Server<perception::CropConfig>::CallbackType crop_f;

	seg_f = boost::bind(&perception::Segmenter::paramsCallback, &segmenter, _1, _2);
	seg_server.setCallback(seg_f);

	//crop_f = boost::bind(&perception::Cropper::paramsCallback, &cropper, _1, _2);
	//crop_server.setCallback(crop_f);


	ros::spin();
	return 0;

}

