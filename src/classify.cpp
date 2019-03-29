#include "perception/classify.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/features/normal_3d.h"
#include "pcl/features/cvfh.h"
//#include "pcl/features/grsd.h"
#include "pcl/features/esf.h"

#include "pcl/common/angles.h"
#include "pcl/visualization/histogram_visualizer.h"
#include "visualization_msgs/Marker.h"

#include "pcl/io/pcd_io.h"

#include "stdio.h"
#include "termios.h"
#include "unistd.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZI PointI;

typedef pcl::PointCloud<pcl::VFHSignature308> PointCloudVFH308;

int main(int argc, char** argv) {
	ros::init(argc, argv, "Classifier");
	ros::NodeHandle nh;

	std::string location = argc > 1 ? argv[1] : ".";

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("object_markers", 1);
	ros::Publisher descriptor_pub = nh.advertise<sensor_msgs::PointCloud2>("CVFH_Descriptors", 1);

	bool RGB;
	ros::param::get("/perception/RGB", RGB);

	if(RGB) {
		perception::Classifier<PointC> classifier(marker_pub, descriptor_pub, location);
		ROS_INFO("Classifying RGB");

		ros::Subscriber sub = nh.subscribe("outlier_removed_cloud", 1,
						&perception::Classifier<PointC>::Callback, &classifier);

		ros::spin();
		return 0;
	}
	else{
		perception::Classifier<PointI> classifier(marker_pub, descriptor_pub, location);
		ROS_INFO("Classifying LIDAR");

		ros::Subscriber sub = nh.subscribe("downsampled_cloud", 1,
						&perception::Classifier<PointI>::Callback, &classifier);

		ros::spin();
		return 0;
	}
}

namespace perception {
	int getch() {
		static struct termios oldt, newt;

		/*tcgetattr gets the parameters of the current terminal
		STDIN_FILENO will tell tcgetattr that it should write the settings
		of stdin to oldt*/
		tcgetattr(STDIN_FILENO, &oldt);

		/*now the settings will be copied*/
		newt = oldt;

		/*ICANON normally takes care that one line at a time will be processed
		that means it will return if it sees a "\n" or an EOF or an EOL*/
		newt.c_lflag &= ~(ICANON | ECHO);

		/*Those new settings will be set to STDIN
		TCSANOW tells tcsetattr to change attributes immediately.*/
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);

		/*Retrieve character*/
		int c = getchar();

		/*restore the old settings*/
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		return c;
	}

	template <class T>
	Classifier<T>::Classifier(const ros::Publisher& marker_pub, 
							  const ros::Publisher& descriptor_pub,
							  const std::string location) 

		: marker_pub_(marker_pub),
		  descriptor_pub_(descriptor_pub) {

		location_ = location;

		f = boost::bind(&perception::Classifier<T>::paramsCallback, this, _1, _2);
		server.setCallback(f);
	}

	template <class T>
	void Classifier<T>::computeNormals(typename pcl::PointCloud<T>::Ptr cloud, 
									   typename pcl::PointCloud<pcl::Normal>::Ptr normals) {
		pcl::NormalEstimation<T, pcl::Normal> norm;

		norm.setInputCloud(cloud);
		norm.setRadiusSearch(radius_limit);

		typename pcl::search::KdTree<T>::Ptr kdtree(new pcl::search::KdTree<T>);
		norm.setSearchMethod(kdtree);
		norm.compute(*normals);
	}

	template <class T>
	void Classifier<T>::CVFH_Descriptors(typename pcl::PointCloud<T>::Ptr cloud,
										 typename pcl::PointCloud<pcl::Normal>::Ptr normals) {
		PointCloudVFH308::Ptr descriptors(new PointCloudVFH308());
		/*
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		pcl::NormalEstimation<T, pcl::Normal> norm;
		norm.setInputCloud(cloud);
		norm.setRadiusSearch(radius_limit);
		typename pcl::search::KdTree<T>::Ptr kdtree(new pcl::search::KdTree<T>);
		norm.setSearchMethod(kdtree);
		norm.compute(*normals);
		*/

		typename pcl::search::KdTree<T>::Ptr kdtree(new pcl::search::KdTree<T>);

		pcl::CVFHEstimation<T, pcl::Normal, pcl::VFHSignature308> cvfh;

		cvfh.setInputCloud(cloud);
		cvfh.setInputNormals(normals);
		cvfh.setSearchMethod(kdtree);
		cvfh.setEPSAngleThreshold(pcl::deg2rad(epsilon_angle));
		cvfh.setCurvatureThreshold(curvature_threshold);
		cvfh.setNormalizeBins(false);
		cvfh.compute(*descriptors);

		std::stringstream filename_ss;
		filename_ss << location_.c_str() << "test_" << idx << ".pcd";

		//ROS_INFO("Arguments passed %s", location_.c_str());

/*
		int c = getch();
		if (c == 's') {
			pcl::io::savePCDFile(filename_ss.str(), *descriptors);
			ROS_INFO("Saving PCD File");
			idx ++;
		}
*/
		//ROS_INFO("CVFH: %ld", descriptors->points.size());

		//---------- Convert and Publish Cloud to Topic ----------//
		sensor_msgs::PointCloud2 msg_out;

		if(RGB) msg_out.header.frame_id = "camera_link";
		else msg_out.header.frame_id = "velodyne";

		pcl::toROSMsg(*descriptors, msg_out);
		descriptor_pub_.publish(msg_out);

		pcl::visualization::PCLHistogramVisualizer viewer;
		if(viewer.addFeatureHistogram(*descriptors, 308)) ROS_INFO("Creating Histogram Visualizer");
		else viewer.updateFeatureHistogram(*descriptors, 308);

		viewer.spin();
	}
/*
	template <class T>
	void Classifier<T>::GRSD_Descriptors(typename pcl::PointCloud<T>::Ptr cloud) {
		pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors(new pcl::GRSDSignature21());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		pcl::NormalEstimation<T, pcl::Normal> norm;
		norm.setInputCloud(cloud);
		norm.setRadiusSearch(radius_limit);

		typename pcl::search::KdTree<T>::Ptr kdtree(new pcl::search::KdTree<T>);
		norm.setSearchMethod(kdtree);
		norm.compute(*normals);

		pcl::CVFHEstimation<T, pcl::Normal, pcl::GRSDSignature21> grsd;
		grsd.setInputCloud(cloud);
		grsd.setInputNormals(normals);
		grsd.setSearchMethod(kdtree);

		grsd.setRadiusSearch(0.05);
		grsd.compute(*descriptors);

		pcl::visualization::PCLHistogramVisualizer viewer;
		viewer.addFeatureHistogram(*descriptors, 308);

		viewer.spin();
	}
*/

	template <class T>
	void Classifier<T>::ESF_Descriptors(typename pcl::PointCloud<T>::Ptr cloud) {
		pcl::ESFEstimation<T, pcl::ESFSignature640> esf;
		pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>());
		
		esf.setInputCloud(cloud);
		esf.compute(*descriptor);

		pcl::visualization::PCLHistogramVisualizer viewer;
		if(viewer.addFeatureHistogram(*descriptor, 640)) ROS_INFO("Creating Histogram Visualizer");
		else viewer.updateFeatureHistogram(*descriptor, 640);

		viewer.spin();
	}

	template <class T>
	void Classifier<T>::paramsCallback(perception::ClassifyConfig &config, uint32_t level) {
		radius_limit = config.radius_limit;
		epsilon_angle = config.epsilon_angle;
		curvature_threshold = config.curvature_threshold;
	}

	template <class T>
	void Classifier<T>::Callback(const sensor_msgs::PointCloud2& msg) {
		typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
		typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		
		pcl::fromROSMsg(msg, *cloud);
		this->computeNormals(cloud, normals);

		//this->CVFH_Descriptors(cloud, normals);
		//this->GRSD_Descriptors(cloud);
		this->ESF_Descriptors(cloud);
	}
}