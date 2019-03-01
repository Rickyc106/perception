#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/DownSampleConfig.h"
#include "dynamic_reconfigure/server.h"

namespace perception {
	class Downsampler {
	public:
		Downsampler(const ros::Publisher& pub);

		void paramsCallback(perception::DownSampleConfig &config, uint32_t level);
		void Callback(const sensor_msgs::PointCloud2& msg);

	private:
		ros::Publisher pub_;

		dynamic_reconfigure::Server<perception::DownSampleConfig> server;
		dynamic_reconfigure::Server<perception::DownSampleConfig>::CallbackType f;
	};
}