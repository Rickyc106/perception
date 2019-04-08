#include "perception/classify.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/features/fpfh.h"
//#include <pcl/features/rsd.h>
#include "pcl/features/3dsc.h"

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

#include "math.h"

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

		ros::Subscriber sub = nh.subscribe("outlier_removed_cloud", 1,
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
		idx = 0;
		esf_idx = 0;

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

	template<class T>
	void Classifier<T>::computeSize(typename pcl::PointCloud<T>::Ptr cloud, double *array) {
		Eigen::Vector4f min_pt, max_pt;
		pcl::getMinMax3D(*cloud, min_pt, max_pt);

		array[0] = (max_pt.z() - min_pt.z());
		array[1] = (max_pt.x() - min_pt.x());
		array[2] = (max_pt.y() - min_pt.y());
		array[3] = cloud->points.size();
	}

	template<class T>
	void Classifier<T>::FPFH_Descriptors(typename pcl::PointCloud<T>::Ptr cloud, 
										 typename pcl::PointCloud<pcl::Normal>::Ptr normals) {
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
		typename pcl::search::KdTree<T>::Ptr kdtree(new pcl::search::KdTree<T>);

		pcl::FPFHEstimation<T, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud(cloud);
		fpfh.setInputNormals(normals);
		fpfh.setSearchMethod(kdtree);
		fpfh.setRadiusSearch(0.05);
		fpfh.compute(*descriptors);

		pcl::visualization::PCLHistogramVisualizer viewer;
		if(viewer.addFeatureHistogram(*descriptors, 33)) {
			ROS_INFO("Creating Histogram Visualizer");
			ROS_INFO("Press s to save histogram");

			int c = getch();
			if (c == 's') {
				//pcl::io::savePCDFile(filename_ss.str(), *descriptors);
				ROS_INFO("Saving PCD File");
				idx ++;
			}

			viewer.spinOnce();
		}
	}
/*
	template<class T>
	void Classifier<T>::RSD_Descriptors(typename pcl::PointCloud<T>::Ptr cloud,
										typename pcl::PointCloud<pcl::Normal>::Ptr normals) {
		pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
		typename pcl::search::KdTree<T>::Ptr kdtree(new pcl::search::KdTree<T>);

		pcl::RSDEstimation<T, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
		rsd.setInputCloud(cloud);
		rsd.setInputNormals(normals);
		rsd.setSearchMethod(kdtree);
		rsd.setRadiusSearch(0.03);
		rsd.setPlaneRadius(plane_radius);
		rsd.setSaveHistograms(false);
		rsd.compute(*descriptors);

		pcl::visualization::PCLHistogramVisualizer viewer;
		if(viewer.addFeatureHistogram(*descriptors, 308)) {
			ROS_INFO("Creating Histogram Visualizer");
			ROS_INFO("Press s to save histogram");

			int c = getch();
			if (c == 's') {
				//pcl::io::savePCDFile(filename_ss.str(), *descriptors);
				ROS_INFO("Saving PCD File");
				idx ++;
			}

			viewer.spinOnce();
		}

	}
*/
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
		if(viewer.addFeatureHistogram(*descriptors, 308)) {
			ROS_INFO("Creating Histogram Visualizer");
			ROS_INFO("Press s to save histogram");

			/*int c = getch();
			if (c == 's') {
				pcl::io::savePCDFile(filename_ss.str(), *descriptors);
				ROS_INFO("Saving PCD File");
				idx ++;
			}*/
		}
		//else viewer.updateFeatureHistogram(*descriptors, 308);

		//viewer.spinOnce();
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
	void Classifier<T>::SC3D_Descriptors(typename pcl::PointCloud<T>::Ptr cloud, 
										 typename pcl::PointCloud<pcl::Normal>::Ptr normals) {
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());
		typename pcl::search::KdTree<T>::Ptr kdtree(new pcl::search::KdTree<T>);

		pcl::ShapeContext3DEstimation<T, pcl::Normal, pcl::ShapeContext1980> sc3d;
		sc3d.setInputCloud(cloud);
		sc3d.setInputNormals(normals);
		sc3d.setSearchMethod(kdtree);
		sc3d.setRadiusSearch(0.05);
		sc3d.setMinimalRadius(0.05 / 10.0);
		sc3d.setPointDensityRadius(0.05 / 5.0);
		sc3d.compute(*descriptors);

/*
		pcl::visualization::PCLHistogramVisualizer viewer;
		if(viewer.addFeatureHistogram(*descriptors, 1980)) {
			ROS_INFO("Creating Histogram Visualizer");
			ROS_INFO("Press s to save histogram");

			int c = getch();
			if (c == 's') {
				//pcl::io::savePCDFile(filename_ss.str(), *descriptors);
				ROS_INFO("Saving PCD File");
				idx ++;
			}

			viewer.spinOnce();
		}
*/
	}

	template <class T>
	void Classifier<T>::ESF_Descriptors(typename pcl::PointCloud<T>::Ptr cloud) {
		pcl::ESFEstimation<T, pcl::ESFSignature640> esf;
		pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>());
		
		esf.setInputCloud(cloud);
		esf.compute(*descriptor);

		std::vector<double> histogram_data;

		for (int i = 0; i < 640; i++) {
			histogram_data.push_back(descriptor->points[0].histogram[i]);
		}

		if(esf_idx == 0) {
			esf_objects.push_back(histogram_data);
			ROS_INFO("Empty Vector of Vectors. Appending 1st Object");

			esf_idx += 1;
			ROS_INFO("New object found! Currently %d objects found: ", esf_idx);
		}
		else {
			/*
			std::vector< std::vector<double> >::const_iterator object;
			std::vector<double>::const_iterator column;

			for(object = esf_objects.begin(); object != esf_objects.end(); object++) {
				for(column = object->begin(); column != object->end(); column++) {
					double diff = (histogram_data[*column] - esf_objects[*object][*column]);
					sum_of_squares += std::pow(diff, diff);

					if(sum_of_squares <= new_object_threshold) {
						esf_objects.push_back(histogram_data);

						esf_idx += 1;
						ROS_INFO("New object found! Currently %d objects found: ", esf_idx);
					}
				}
			}
			*/

			for(int object = 0; object != esf_objects.size(); object++) {
				double sum_of_squares = 0;

				for (int column = 0; column != esf_objects[object].size(); column++) {
					double diff = (histogram_data[column] - esf_objects[object][column]);
					sum_of_squares += std::pow(diff, 2);

					if(sum_of_squares >= new_object_threshold) {
						//ROS_INFO("1st Condition SS: %f \t object: %d", sum_of_squares, object);
						if(object == (esf_objects.size() - 1)) {
							//ROS_INFO("2nd Condition SS: %f \t object: %d", sum_of_squares, object);
							esf_objects.push_back(histogram_data);

							//ROS_INFO("sum of squares %f", sum_of_squares);
							esf_idx += 1;
							ROS_INFO("New object found! Currently %d objects found: ", esf_idx);
						}
					}
				}

				ROS_INFO("Current object compared to object %d: Sum of squares %f", object, sum_of_squares);

			}
		}

		/*
		pcl::visualization::PCLHistogramVisualizer viewer;
		if(viewer.addFeatureHistogram(*descriptor, 640)) ROS_INFO("Creating Histogram Visualizer");
		else viewer.updateFeatureHistogram(*descriptor, 640);

		viewer.spin();
		*/
	}

	template <class T>
	void Classifier<T>::paramsCallback(perception::ClassifyConfig &config, uint32_t level) {
		radius_limit = config.radius_limit;
		epsilon_angle = config.epsilon_angle;
		curvature_threshold = config.curvature_threshold;
		plane_radius = config.plane_radius;
		new_object_threshold = config.new_object_threshold;
	}

	template <class T>
	void Classifier<T>::Callback(const sensor_msgs::PointCloud2& msg) {
		typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
		typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		
		pcl::fromROSMsg(msg, *cloud);
		this->computeNormals(cloud, normals);

		double size_array[4];
		this->computeSize(cloud, size_array);

/*
		ROS_INFO("Length: %f", size_array[0]);
		ROS_INFO("Width: %f", size_array[1]);
		ROS_INFO("Height: %f", size_array[2]);
		ROS_INFO("Point Cloud Size: %f", size_array[3]);
*/

		//this->CVFH_Descriptors(cloud, normals);
		//this->GRSD_Descriptors(cloud);
		this->ESF_Descriptors(cloud);
		//this->FPFH_Descriptors(cloud, normals);
		//this->SC3D_Descriptors(cloud, normals);
	}
}