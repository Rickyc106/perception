#include "perception/statistical_removal.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;

int main(int argc, char** argv) {
	ros::init(argc, argv, "StatRemover");
	ros::NodeHandle nh;

	ros::Publisher filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("outlier_removed_cloud", 1);
	
	bool RGB;
	ros::param::get("/perception/RGB", RGB);

	if(RGB){
		perception::StatRemover<PointC> stat_remover(filtered_pub);

		ros::Subscriber sub = nh.subscribe("clustered_objects", 1,
						&perception::StatRemover<PointC>::Callback, &stat_remover);

		ros::spin();
		return 0;
	}
	else{
		perception::StatRemover<PointI> stat_remover(filtered_pub);

		ros::Subscriber sub = nh.subscribe("clustered_objects", 1,
						&perception::StatRemover<PointI>::Callback, &stat_remover);

		ros::spin();
		return 0;
	}

}

namespace perception {
	template <class T>
	StatRemover<T>::StatRemover(const ros::Publisher& pub) : pub_(pub) {
		f = boost::bind(&perception::StatRemover<T>::paramsCallback, this, _1, _2);
		server.setCallback(f);
	}

	template <class T>
	void StatRemover<T>::paramsCallback(perception::StatRemoveConfig &config, uint32_t level) {
		mean_k = config.mean_k;
		std_dev_multiplier_thresh = config.std_dev_multiplier_thresh;
	}

	template <class T>
	void StatRemover<T>::Callback(const sensor_msgs::PointCloud2& msg) {
		typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
		pcl::fromROSMsg(msg, *cloud);

		typename pcl::PointCloud<T>::Ptr filtered_cloud(new pcl::PointCloud<T>());
		pcl::StatisticalOutlierRemoval<T> sor;

		ros::param::get("mean_k", mean_k);
		ros::param::get("std_dev_multiplier_thresh", std_dev_multiplier_thresh);

		ros::param::get("/perception/RGB", RGB);

		sor.setInputCloud(cloud);
		sor.setMeanK(mean_k);
		sor.setStddevMulThresh(std_dev_multiplier_thresh);
		sor.filter(*filtered_cloud);

		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*filtered_cloud, msg_out);

		if(RGB) msg_out.header.frame_id = "camera_link";
		else msg_out.header.frame_id = "velodyne";

		pub_.publish(msg_out);
	}
}

