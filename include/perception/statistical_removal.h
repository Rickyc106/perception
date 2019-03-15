#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/StatRemoveConfig.h"
#include "dynamic_reconfigure/server.h"

namespace perception {
	template <class T>
	class StatRemover {
		public:
			StatRemover(const ros::Publisher& pub);

			void paramsCallback(perception::StatRemoveConfig &config, uint32_t level);
			void Callback(const sensor_msgs::PointCloud2& msg);

		private:
			ros::Publisher pub_;

			dynamic_reconfigure::Server<perception::StatRemoveConfig> server;
			dynamic_reconfigure::Server<perception::StatRemoveConfig>::CallbackType f;

			double mean_k;
			double std_dev_multiplier_thresh;

			bool RGB;
	};
}