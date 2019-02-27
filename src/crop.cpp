#include "ros/ros.h"
#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

double min_x, min_y, min_z, max_x, max_y, max_z;

namespace perception {
	Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

	void Cropper::SetParams() {
		/*
		ros::param::param("crop_min_x", min_x, 0.3);
		ros::param::param("crop_min_y", min_y, -1.0);
		ros::param::param("crop_min_z", min_z, 0.5);
		ros::param::param("crop_max_x", max_x, 0.9);
		ros::param::param("crop_max_y", max_y, 1.0);
		ros::param::param("crop_max_z", max_z, 1.5);

		Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
		Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
		*/
	}

	void Cropper::Crop() {

	}

	void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
		PointCloudC::Ptr cloud(new PointCloudC());
		pcl::fromROSMsg(msg, *cloud);
		ROS_INFO("Got point cloud with %ld points", cloud->size());

		PointCloudC::Ptr cropped_cloud(new PointCloudC());
		//SetParams();

/*
		ros::param::param("crop_min_x", min_x, 0.3);
		ros::param::param("crop_min_y", min_y, -1.0);
		ros::param::param("crop_min_z", min_z, 0.5);
		ros::param::param("crop_max_x", max_x, 0.9);
		ros::param::param("crop_max_y", max_y, 1.0);
		ros::param::param("crop_max_z", max_z, 1.5);
*/

		ros::param::get("crop_min_x", min_x);
		ros::param::get("crop_min_y", min_y);
		ros::param::get("crop_min_z", min_z);
		ros::param::get("crop_max_x", max_x);
		ros::param::get("crop_max_y", max_y);
		ros::param::get("crop_max_z", max_z);

		ROS_INFO("Min x: %f", min_x);

		Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
		Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);

		pcl::CropBox<PointC> crop;
		crop.setInputCloud(cloud);
		crop.setMin(min_pt);
		crop.setMax(max_pt);
		crop.filter(*cropped_cloud);
		ROS_INFO("Cropped to %ld points", cropped_cloud->size());

		PointCloudC::Ptr downsampled_cloud(new PointCloudC());
		pcl::VoxelGrid<PointC> vox;
		vox.setInputCloud(cloud);

		double voxel_size;
		ros::param::param("voxel_size", voxel_size, 0.1);
		vox.setLeafSize(voxel_size, voxel_size, voxel_size);
		vox.filter(*downsampled_cloud);

		sensor_msgs::PointCloud2 msg_out;
		//pcl::toROSMsg(*cropped_cloud, msg_out);
		pcl::toROSMsg(*downsampled_cloud, msg_out);
		pub_.publish(msg_out);
	}
}