#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"

#include "perception/ClassifyConfig.h"
#include "dynamic_reconfigure/server.h"

namespace perception {
	template <class T>
	class Classifier {
	public:
		Classifier(const ros::Publisher& marker_pub, 
				   const ros::Publisher& descriptor_pub);

		void CVFH_Descriptors(typename pcl::PointCloud<T>::Ptr cloud);
		void GRSD_Descriptors(typename pcl::PointCloud<T>::Ptr cloud);

		void paramsCallback(perception::ClassifyConfig &config, uint32_t level);
		void Callback(const sensor_msgs::PointCloud2& msg);

	private:
		ros::Publisher marker_pub_;
		ros::Publisher descriptor_pub_;

		dynamic_reconfigure::Server<perception::ClassifyConfig> server;
		dynamic_reconfigure::Server<perception::ClassifyConfig>::CallbackType f;

		double radius_limit;
		double epsilon_angle;
		double curvature_threshold;

		bool RGB;
	};
}