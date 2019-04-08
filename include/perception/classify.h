#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"

#include "perception/ClassifyConfig.h"
#include "dynamic_reconfigure/server.h"

#include "vector"

namespace perception {
	int getch();

	template <class T>
	class Classifier {
	public:
		Classifier(const ros::Publisher& marker_pub, 
				   const ros::Publisher& descriptor_pub,
				   const std::string location);

		void computeNormals(typename pcl::PointCloud<T>::Ptr cloud,
							typename pcl::PointCloud<pcl::Normal>::Ptr normals);
		void computeSize(typename pcl::PointCloud<T>::Ptr cloud, double *array);

		// Local Descriptors
		void FPFH_Descriptors(typename pcl::PointCloud<T>::Ptr cloud,
							  typename pcl::PointCloud<pcl::Normal>::Ptr normals);
		void RSD_Descriptors(typename pcl::PointCloud<T>::Ptr cloud,
							 typename pcl::PointCloud<pcl::Normal>::Ptr normals);
		void SC3D_Descriptors(typename pcl::PointCloud<T>::Ptr cloud,
					 		  typename pcl::PointCloud<pcl::Normal>::Ptr normals);

		// Global Descriptors
		void CVFH_Descriptors(typename pcl::PointCloud<T>::Ptr cloud,
							  typename pcl::PointCloud<pcl::Normal>::Ptr normals);
		void GRSD_Descriptors(typename pcl::PointCloud<T>::Ptr cloud);
		void ESF_Descriptors(typename pcl::PointCloud<T>::Ptr cloud);

		// Dynamic Reconfigure
		void paramsCallback(perception::ClassifyConfig &config, uint32_t level);

		// Subscriber callback to original point cloud
		void Callback(const sensor_msgs::PointCloud2& msg);

	private:
		ros::Publisher marker_pub_;
		ros::Publisher descriptor_pub_;

		std::string location_;
		int idx;
		int esf_idx;

		std::vector<std::vector<double> > esf_objects;
		std::vector<double> test_vec;

		dynamic_reconfigure::Server<perception::ClassifyConfig> server;
		dynamic_reconfigure::Server<perception::ClassifyConfig>::CallbackType f;

		double radius_limit;
		double epsilon_angle;
		double curvature_threshold;
		double plane_radius;
		double new_object_threshold;

		bool RGB;
	};
}