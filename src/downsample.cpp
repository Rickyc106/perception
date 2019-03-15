#include "perception/downsample.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;

int main(int argc, char** argv) {
	ros::init(argc, argv, "Downsampler");
	ros::NodeHandle nh;

	ros::Publisher downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1);
	
	bool RGB;
	ros::param::get("/perception/RGB", RGB);

	if(RGB){
		perception::Downsampler<PointC> downsampler(downsample_pub);

		ros::Subscriber sub = nh.subscribe("cropped_cloud", 1,
					&perception::Downsampler<PointC>::Callback, &downsampler);

		ros::spin();
		return 0;
	}
	else{
		perception::Downsampler<PointI> downsampler(downsample_pub);

		ros::Subscriber sub = nh.subscribe("cropped_cloud", 1,
					&perception::Downsampler<PointI>::Callback, &downsampler);

		ros::spin();
		return 0;
	}
}

namespace perception {
	template <class T>
	Downsampler<T>::Downsampler(const ros::Publisher& pub) : pub_(pub) {
		f = boost::bind(&perception::Downsampler<T>::paramsCallback, this, _1, _2);
		server.setCallback(f);
	}

	template <class T>
	void Downsampler<T>::paramsCallback(perception::DownSampleConfig &config, uint32_t level) {
		voxel_size = config.voxel_size;
	}

	template <class T>
	void Downsampler<T>::Callback(const sensor_msgs::PointCloud2& msg) {
		typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
		pcl::fromROSMsg(msg, *cloud);
		//ROS_INFO("Downsampler got point cloud with %ld points", cloud->size());

		typename pcl::PointCloud<T>::Ptr downsampled_cloud(new pcl::PointCloud<T>);
		pcl::VoxelGrid<T> vox;
		vox.setInputCloud(cloud);

		ros::param::get("voxel_size", voxel_size);
		vox.setLeafSize(voxel_size, voxel_size, voxel_size);
		vox.filter(*downsampled_cloud);

		ros::param::get("/perception/RGB", RGB);

		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*downsampled_cloud, msg_out);

		if(RGB) msg_out.header.frame_id = "camera_link";
		else msg_out.header.frame_id = "velodyne";

		pub_.publish(msg_out);
	} 
}