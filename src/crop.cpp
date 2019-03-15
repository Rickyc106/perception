#include "perception/crop.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;

int main(int argc, char** argv) {
	ros::init(argc, argv, "Cropper");
	ros::NodeHandle nh;

	ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1);

	bool RGB;
	ros::param::get("/perception/RGB", RGB);

	if(RGB) {
		perception::Cropper<PointC> cropper(crop_pub);

		ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1,
					&perception::Cropper<PointC>::Callback, &cropper);

		ros::spin();
		return 0;
	}
	else{
		perception::Cropper<PointI> cropper(crop_pub);

		ros::Subscriber sub = nh.subscribe("velodyne_points", 1,
					&perception::Cropper<PointI>::Callback, &cropper);

		ros::spin();
		return 0;
	}
}

namespace perception {
	//-- Default Cropper Constructor --//
	template <class T>
	Cropper<T>::Cropper(const ros::Publisher& pub) : pub_(pub) {
		f = boost::bind(&perception::Cropper<T>::paramsCallback, this, _1, _2);
		server.setCallback(f); 
	}

	//-- Dynamic Reconfigure Callback --//
	template <class T>
	void Cropper<T>::paramsCallback(perception::CropConfig &config, uint32_t level) {
		min_x = config.crop_min_x;
		min_y = config.crop_min_y;
		min_z = config.crop_min_z;
		max_x = config.crop_max_x;
		max_y = config.crop_max_y;
		max_z = config.crop_max_z;
	}

	//-- Cropper Callback --//
	template <class T>
	void Cropper<T>::Callback(const sensor_msgs::PointCloud2& msg) {
		//PointCloudC::Ptr cloud(new PointCloudC());
		typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
		pcl::fromROSMsg(msg, *cloud);
		//ROS_INFO("Cropper got point cloud with %ld points", cloud->size());

		//PointCloudC::Ptr cropped_cloud(new PointCloudC());
		typename pcl::PointCloud<T>::Ptr cropped_cloud(new pcl::PointCloud<T>());

		ros::param::get("crop_min_x", min_x);
		ros::param::get("crop_min_y", min_y);
		ros::param::get("crop_min_z", min_z);
		ros::param::get("crop_max_x", max_x);
		ros::param::get("crop_max_y", max_y);
		ros::param::get("crop_max_z", max_z);

		ros::param::get("/perception/RGB", RGB);

		//ROS_INFO("Min x: %f", min_x);

		Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
		Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);

		pcl::CropBox<T> crop;
		crop.setInputCloud(cloud);
		crop.setMin(min_pt);
		crop.setMax(max_pt);
		crop.filter(*cropped_cloud);
		//ROS_INFO("Cropped to %ld points", cropped_cloud->size());

		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*cropped_cloud, msg_out);

		if(RGB) msg_out.header.frame_id = "camera_link";
		else msg_out.header.frame_id = "velodyne";

		//pcl::toROSMsg(*downsampled_cloud, msg_out);
		pub_.publish(msg_out);
	}
}