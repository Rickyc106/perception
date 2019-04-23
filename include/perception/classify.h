#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"

#include "perception/ClassifyConfig.h"
#include "dynamic_reconfigure/server.h"

#include "vector"

// Testing
#include "perception/indices.h"
#include "perception/PointCloudArray.h"

namespace perception {
	int getch();

	// Add / Remove Descriptors here
	struct DESCRIPTORS {
		//string namespace;
		int id;

		double x_size;
		double y_size;
		double z_size;
		double cloud_size;

		std::vector<double> esf_attributes; 
	};

	template <class T>
	class Classifier {
	public:
		Classifier(const ros::Publisher& marker_pub, 
				   const ros::Publisher& object_pub,
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
		std::vector<double> ESF_Descriptors(typename pcl::PointCloud<T>::Ptr cloud);

		void viewHistogram();

		void classify(DESCRIPTORS new_object);

		// Dynamic Reconfigure
		void paramsCallback(perception::ClassifyConfig &config, uint32_t level);

		// Subscriber callback to original point cloud
		//void Callback(const sensor_msgs::PointCloud2& msg);
		//void cloudCallback(const sensor_msgs::PointCloud2& msg);
		//void indicesCallback(const perception::indices& msg);
		void Callback(const perception::PointCloudArray& msg);

	private:
		ros::Publisher marker_pub_;
		ros::Publisher object_pub_;

		std::string location_;
		int idx;
		int esf_idx;

		//std::vector<std::vector<double> > esf_objects;
		std::vector<DESCRIPTORS> classified_objects;
		std::vector<double> activation_list;
		std::vector<double> test_vec;

		std::vector<int> indices;

		dynamic_reconfigure::Server<perception::ClassifyConfig> server;
		dynamic_reconfigure::Server<perception::ClassifyConfig>::CallbackType f;

		double radius_limit;
		double epsilon_angle;
		double curvature_threshold;
		double plane_radius;
		double size_weight;
		double esf_weight;
		double lower_activation_threshold;
		double upper_activation_threshold;
		int object;

		bool RGB;
		bool zero_obj_found;
	};
}