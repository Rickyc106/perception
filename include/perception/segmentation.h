#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/point_types.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_msgs/PointIndices.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "perception/SegmentationConfig.h"
#include "dynamic_reconfigure/server.h"

namespace perception {
	template <class T>
	class Segmenter {
		public:
			Segmenter(const ros::Publisher& table_pub, 
					  const ros::Publisher& marker_pub,
					  const ros::Publisher& object_pub,
					  const ros::Publisher& object_indices_pub);

			// TO-DO
			// Reify indices to point cloud
			// Then publish point cloud to topic
			void indicesToPointCloud(pcl::PointIndices::Ptr indices,
									 typename pcl::PointCloud<T>::Ptr subset_cloud);
			
			void SegmentSurfaceFromNormals(typename pcl::PointCloud<T>::Ptr cloud,
								pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, 
								pcl::PointIndices::Ptr indices, 
								typename pcl::PointCloud<T>::Ptr subset_cloud);


			// Finds the largest horizontal surface in the given point cloud.
			// This is useful for adding a collision object to MoveIt.
			//
			// Args:
			//  cloud: The point cloud to extract a surface from.
			//  indices: The indices of points in the point cloud that correspond to the
			//    surface. Empty if no surface was found.
			void SegmentSurfaceFromPerpendicular(typename pcl::PointCloud<T>::Ptr cloud,
								pcl::PointIndices::Ptr indices,
								typename pcl::PointCloud<T>::Ptr subset_cloud);

			// Computes the axis-aligned bounding box of a point cloud.
			//
			// Args:
			//  cloud: The point cloud
			//  pose: The output pose. Because this is axis-aligned, the orientation is just
			//    the identity. The position refers to the center of the box.
			//  dimensions: The output dimensions, in meters.
			void GetAxisAlignedBoundingBox(typename pcl::PointCloud<T>::Ptr cloud,
			                               geometry_msgs::Pose* pose,
			                               geometry_msgs::Vector3* scale);

			// Finds objects on segmented surface using Euclidean Clustering
			void SegmentCylinder(typename pcl::PointCloud<T>::Ptr cloud,
								 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, 
								 pcl::PointIndices::Ptr table_inliers,
								 typename pcl::PointCloud<T>::Ptr object_cloud);

			void SegmentClusters(typename pcl::PointCloud<T>::Ptr cloud,
								 typename pcl::PointCloud<T>::Ptr object_cloud);

			void SegmentClusters(typename pcl::PointCloud<T>::Ptr cloud,
								 pcl::PointIndices::Ptr table_inliers,
								 typename pcl::PointCloud<T>::Ptr object_cloud);

			void GetObjectMarkers(typename pcl::PointCloud<T>::Ptr cloud,
								  std::vector<pcl::PointIndices> object_indices);

			void paramsCallback(perception::SegmentationConfig &config, uint32_t level);
			void Callback(const sensor_msgs::PointCloud2& msg);
		
		private:
			ros::Publisher table_pub_;
			ros::Publisher marker_pub_;
			ros::Publisher object_pub_;
			ros::Publisher object_indices_pub_;
			
			dynamic_reconfigure::Server<perception::SegmentationConfig> server;
			dynamic_reconfigure::Server<perception::SegmentationConfig>::CallbackType f;

			double distance_threshold;
			std::string axis_param;
			double epsilon_angle;
			double distance_above_plane;

			double normal_distance_weight;
			double max_iterations;

			double radius_limit;

			double cluster_tolerance;
			int min_cluster_size;
			int max_cluster_size;

			bool RGB;
	};
}