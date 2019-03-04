#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/CropConfig.h"
#include "dynamic_reconfigure/server.h"

namespace perception {
	class Cropper {
	public:
		Cropper(const ros::Publisher& pub);

		void paramsCallback(perception::CropConfig &config, uint32_t level);
		void Callback(const sensor_msgs::PointCloud2& msg);

	private:
		ros::Publisher pub_;

		dynamic_reconfigure::Server<perception::CropConfig> server;
		dynamic_reconfigure::Server<perception::CropConfig>::CallbackType f;

		double min_x, min_y, min_z;
		double max_x, max_y, max_z;
	};
}
