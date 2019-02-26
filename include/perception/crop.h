#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception {
	class Cropper {
	public:
		Cropper(const ros::Publisher& pub);
		void SetParams();
		void Crop();
		void Callback(const sensor_msgs::PointCloud2& msg);

	private:
		ros::Publisher pub_;
	};
}
