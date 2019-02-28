#include "perception/crop.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
	int main(int argc, char** argv) {
		ros::init(argc, argv, "Downsampler");
		ros::NodeHandle nh;

		ros::Publisher downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1);
		Downsampler downsampler(downsample_pub);

		ros::Subscriber sub = nh.subscribe("cropped_cloud", 1,
						&Downsampler::Callback, &downsampler);

		ros::spin();
		return 0;
	}

	Downsampler::Downsampler(const sensor_msgs& pub) : pub_(pub) {
		f = boost::bind(&perception::Downsampler::paramsCallback, this, _1, _2);
		server.setCallback(f);
	}

	void Downsampler::paramsCallback(perception::DownSampleConfig, uint32_t level) {
		ROS_INFO("Reconfigure Request: %f", config.voxel_size);
	}

	void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
		PointCloudC::Ptr cloud(new PointCloudC());
		pcl::fromROSMsg(msg, *cloud);
		ROS_INFO("Downsampler got point cloud with %ld points", cloud->size());

		PointCloudC::Ptr downsampled_cloud(new PointCloudC());
		pcl::VoxelGrid<PointC> vox;
		vox.setInputCloud(cloud);

		double voxel_size;
		ros::param::get("voxel_size", voxel_size);
		vox.setLeafSize(voxel_size, voxel_size, voxel_size);
		vox.filter(*downsampled_cloud);

		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*downsampled_cloud, msg_out);
		pub_.publish(msg_out);
	} 
}