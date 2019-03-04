#include "perception/segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"
#include "visualization_msgs/Marker.h"

// Libraries for RANSAC Algorithm
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

// Reify point clud from indices
#include "pcl/filters/extract_indices.h"

// Euclidean Cluster Extraction
#include "pcl/segmentation/extract_clusters.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

int main(int argc, char** argv) {
	ros::init(argc, argv, "Segmenter");
	ros::NodeHandle nh;

	ros::Publisher table_pub = nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visual_marker", 1);
	ros::Publisher object_pub = nh.advertise<sensor_msgs::PointCloud2>("clustered_objects", 1);

	perception::Segmenter segmenter(table_pub, marker_pub, object_pub);

	ros::Subscriber sub = nh.subscribe("downsampled_cloud", 1,
					&perception::Segmenter::Callback, &segmenter);

	ros::spin();
	return 0;
}

namespace perception {
	//-- Default Segmenter Constructor --//
	Segmenter::Segmenter(const ros::Publisher& surface_points_pub, 
						 const ros::Publisher& marker_pub,
						 const ros::Publisher& object_pub)
		: surface_points_pub_(surface_points_pub), 
		  marker_pub_(marker_pub),
		  object_pub_(object_pub) {

		f = boost::bind(&perception::Segmenter::paramsCallback, this, _1, _2);
		server.setCallback(f);
	}

	//-- Helper Segment Surface Function --//
	void Segmenter::SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, 
		PointCloudC::Ptr subset_cloud) {

		ros::param::get("distance_threshold", distance_threshold);
		ros::param::get("axis", axis_param);
		ros::param::get("epsilon_angle", epsilon_angle);
		
		ros::param::get("distance_above_plane", distance_above_plane);
	
		pcl::PointIndices indices_internal;
		pcl::SACSegmentation<PointC> seg;
		seg.setOptimizeCoefficients(true);

		// Search for a plane perpendicular to some axis (specified below).
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);

		// Set the distance to the plane for a point to be an inlier.
		seg.setDistanceThreshold(distance_threshold);
		seg.setInputCloud(cloud);

		seg.setMaxIterations(500); // Default is 50

		Eigen::Vector3f axis;

		if (axis_param == "X" || axis_param == "x") axis << 1.0, 0, 0;
		else if (axis_param == "Y" || axis_param == "y") axis << 0, 1.0, 0;
		else if (axis_param == "Z" || axis_param == "z") axis << 0, 0, 1.0;

		seg.setAxis(axis);
		seg.setEpsAngle(pcl::deg2rad(epsilon_angle));

		// coeff contains the coefficients of the plane:
		// ax + by + cz + d = 0
		pcl::ModelCoefficients coeff;
		seg.segment(indices_internal, coeff);

		for (int i = 0; i < cloud->size(); i++) {
			const PointC& pt = cloud->points[i];
			float val = coeff.values[0] * pt.x + 
						coeff.values[1] * pt.y + 
						coeff.values[2] * pt.z + 
						coeff.values[3];
			if (val <= distance_above_plane) indices->indices.push_back(i);
		}

		*indices = indices_internal;

		if (indices->indices.size() == 0) {
			ROS_ERROR("Unable to find surface.");
			return;
		}

		pcl::ExtractIndices<PointC> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(indices);
		extract.filter(*subset_cloud);
	}

	//-- Helper Marker Function --//
	void Segmenter::GetAxisAlignedBoundingBox(PointCloudC::Ptr cloud, geometry_msgs::Pose* pose,
		geometry_msgs::Vector3* scale) {

		Eigen::Vector4f min_pt, max_pt;
		pcl::getMinMax3D(*cloud, min_pt, max_pt);

		pose->position.x = (max_pt.z() + min_pt.z()) / 2;
		pose->position.y = (max_pt.x() + min_pt.x()) / 2;
		pose->position.z = (max_pt.y() + min_pt.y()) / 2;
		pose->orientation.w = 1;

		scale->x = max_pt.z() - min_pt.z();
		scale->y = max_pt.x() - min_pt.x();
		scale->z = max_pt.y() - min_pt.y();

		ROS_INFO("Position: %f %f %f", min_pt.x(), min_pt.y(), min_pt.z());
	}

	//-- Helper Segement Surface Object Function --//
	void Segmenter::SegmentSurfaceObjects(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, 
							   std::vector<pcl::PointIndices>* object_indices) {
		pcl::ExtractIndices<PointC> extract;
		pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
		extract.setInputCloud(cloud);
		extract.setIndices(indices);
		extract.setNegative(true);
		extract.filter(above_surface_indices->indices);

		ros::param::get("cluster_tolerance", cluster_tolerance);
		ros::param::get("min_cluster_size", min_cluster_size);
		ros::param::get("max_cluster_size", max_cluster_size);

		pcl::EuclideanClusterExtraction<PointC> euclid;
		euclid.setInputCloud(cloud);
		euclid.setIndices(above_surface_indices);
		euclid.setClusterTolerance(cluster_tolerance);
		euclid.setMinClusterSize(min_cluster_size);
		euclid.setMaxClusterSize(max_cluster_size);
		euclid.extract(*object_indices);

		ROS_INFO("Found %ld objects", object_indices->size());
		ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());
	}

	//-- Dynamic Reconfigure Callback --//
	void Segmenter::paramsCallback(perception::SegmentationConfig &config, uint32_t level) {
		distance_threshold = config.distance_threshold;
		axis_param = config.axis;
		epsilon_angle = config.epsilon_angle;
		distance_above_plane = config.distance_above_plane;

		cluster_tolerance = config.cluster_tolerance;
		min_cluster_size = config.min_cluster_size;
		max_cluster_size = config.max_cluster_size;
	}

	//-- Segmenter Callback --//
	void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
		PointCloudC::Ptr cloud(new PointCloudC());
		pcl::fromROSMsg(msg, *cloud);

		pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
		PointCloudC::Ptr subset_cloud(new PointCloudC);

		this->SegmentSurface(cloud, table_inliers, subset_cloud);

		visualization_msgs::Marker table_marker;
		table_marker.header.frame_id = "camera_link";
		table_marker.type = visualization_msgs::Marker::CUBE;

		this->GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
		table_marker.color.r = 1;
		table_marker.color.a = 0.8;
		marker_pub_.publish(table_marker);

		std::vector<pcl::PointIndices> object_indices;
		this->SegmentSurfaceObjects(subset_cloud, table_inliers, &object_indices);

		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*subset_cloud, msg_out);
		surface_points_pub_.publish(msg_out);
	}
}