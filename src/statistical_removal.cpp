#include "perception/statistical_removal.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

int main(int argc, char** argv) {
	ros::init(argc, argv, "StatRemover");
	ros::NodeHandle nh;

	ros::Publisher filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("outlier_removed_cloud", 1);
	perception::StatRemover stat_remover(filtered_pub);

	ros::Subscriber sub = nh.subscribe("table_cloud", 1,
					&perception::StatRemover::Callback, &stat_remover);

	ros::spin();
	return 0;
}

namespace perception {
	StatRemover::StatRemover(const ros::Publisher& pub) : pub_(pub) {
		f = boost::bind(&perception::StatRemover::paramsCallback, this, _1, _2);
		server.setCallback(f);
	}

	void StatRemover::paramsCallback(perception::StatRemoveConfig &config, uint32_t level) {
		mean_k = config.mean_k;
		std_dev_multiplier_thresh = config.std_dev_multiplier_thresh;
	}

	void StatRemover::Callback(const sensor_msgs::PointCloud2& msg) {
		PointCloudC::Ptr cloud(new PointCloudC());
		pcl::fromROSMsg(msg, *cloud);

		PointCloudC::Ptr filtered_cloud(new PointCloudC());
		pcl::StatisticalOutlierRemoval<PointC> sor;

		ros::param::get("mean_k", mean_k);
		ros::param::get("std_dev_multiplier_thresh", std_dev_multiplier_thresh);

		sor.setInputCloud(cloud);
		sor.setMeanK(mean_k);
		sor.setStddevMulThresh(std_dev_multiplier_thresh);
		sor.filter(*filtered_cloud);

		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*filtered_cloud, msg_out);
		pub_.publish(msg_out);
	}
}

