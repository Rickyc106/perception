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

// Testing
#include "perception/indices.h"

// Reify point clud from indices
#include "pcl/filters/extract_indices.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZI PointI;

typedef pcl::PointCloud<pcl::VFHSignature308> PointCloudVFH308;

int main(int argc, char** argv) {
	ros::init(argc, argv, "Classifier");
	ros::NodeHandle nh;

	std::string location = argc > 1 ? argv[1] : ".";

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("object_markers", 1);
	ros::Publisher object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_debug", 1);

	bool RGB;
	ros::param::get("/perception/RGB", RGB);

	if(RGB) {
		perception::Classifier<PointC> classifier(marker_pub, object_pub, location);
		ROS_INFO("Classifying RGB");

		ros::Subscriber sub = nh.subscribe("clustered_objects", 1,
						&perception::Classifier<PointC>::Callback, &classifier);

		//ros::Subscriber indices_sub = nh.subscribe("object_indices", 1,
		//				&perception::Classifier<PointC>::indicesCallback, &classifier);
		//ros::Subscriber cloud_sub = nh.subscribe("downsampled_cloud", 1,
		//				&perception::Classifier<PointC>::cloudCallback, &classifier);

		ros::spin();
		return 0;
	}
	else{
		perception::Classifier<PointI> classifier(marker_pub, object_pub, location);
		ROS_INFO("Classifying LIDAR");

		ros::Subscriber sub = nh.subscribe("clustered_objects", 1,
						&perception::Classifier<PointI>::Callback, &classifier);

		//ros::Subscriber indices_sub = nh.subscribe("object_indices", 1,
		//				&perception::Classifier<PointI>::indicesCallback, &classifier);
		//ros::Subscriber cloud_sub = nh.subscribe("downsampled_cloud", 1,
		//				&perception::Classifier<PointI>::cloudCallback, &classifier);

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
							  const ros::Publisher& object_pub,
							  const std::string location) 

		: marker_pub_(marker_pub),
		  object_pub_(object_pub) {

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

		array[0] = (max_pt.x() - min_pt.x());
		array[1] = (max_pt.y() - min_pt.y());
		array[2] = (max_pt.z() - min_pt.z());
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
		object_pub_.publish(msg_out);

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
	std::vector<double> Classifier<T>::ESF_Descriptors(typename pcl::PointCloud<T>::Ptr cloud) {
		pcl::ESFEstimation<T, pcl::ESFSignature640> esf;
		pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>());
		
		esf.setInputCloud(cloud);
		esf.compute(*descriptor);

		std::vector<double> histogram_data;

		for (int i = 0; i < 640; i++) {
			histogram_data.push_back(descriptor->points[0].histogram[i]);
		}

		return histogram_data;

		//double size_array[4];
		//this->computeSize(cloud, size_array);

		/*if(esf_idx == 0) {
			ROS_INFO("Empty Vector of Vectors. Appending 1st Object");
			classified_objects.push_back(DESCRIPTORS());
			
			//classified_objects[esf_idx].namespace = "perception";
			classified_objects[esf_idx].id = esf_idx;

			classified_objects[esf_idx].x_size = size_array[0];
			classified_objects[esf_idx].y_size = size_array[1];
			classified_objects[esf_idx].z_size = size_array[2];
			classified_objects[esf_idx].cloud_size = size_array[3];

			classified_objects[esf_idx].esf_attributes = histogram_data;

			esf_idx += 1;
			ROS_INFO("New object found! Currently %d objects found: ", esf_idx);
		}*/
		//else {
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

			/*for(int object = 0; object != classified_objects.size(); object++) {
				double sum_of_squares = 0;

				for (int column = 0; column != histogram_data.size(); column++) {
					double diff = (histogram_data[column] - classified_objects[object].esf_attributes[column]);
					sum_of_squares += std::pow(diff, 2);

					if(sum_of_squares >= esf_threshold) {
						//ROS_INFO("1st Condition SS: %f \t object: %d", sum_of_squares, object);
						if(object == (classified_objects.size() - 1)) {
							//ROS_INFO("2nd Condition SS: %f \t object: %d", sum_of_squares, object);

							classified_objects[object].x_size = size_array[0];	// Append length (x)
							classified_objects[object].y_size = size_array[1];	// Append width (y)
							classified_objects[object].z_size = size_array[2];	// Append height (z)
							classified_objects[object].cloud_size = size_array[3];	// Append Point cloud size

							classified_objects[object].esf_attributes = histogram_data;	// Append ESF Histogram Data

							//ROS_INFO("sum of squares %f", sum_of_squares);
							esf_idx += 1;
							ROS_INFO("New object found! Currently %d objects found: ", esf_idx);
						}
					}
				}

				ROS_INFO("Current object compared to object %d: Sum of squares %f", object, sum_of_squares);

			}
		}*/

		/*
		pcl::visualization::PCLHistogramVisualizer viewer;
		if(viewer.addFeatureHistogram(*descriptor, 640)) ROS_INFO("Creating Histogram Visualizer");
		else viewer.updateFeatureHistogram(*descriptor, 640);

		viewer.spin();
		*/
	}

	template <class T>
	void Classifier<T>::classify(DESCRIPTORS new_object) {
		if(esf_idx == 0) {
			ROS_INFO("Empty Vector of Vectors. Appending 1st Object");
			classified_objects.push_back(new_object);

			esf_idx += 1;
			ROS_INFO("New object found! Currently %d objects found: ", esf_idx);
		}
		else {
			for(int object = 0; object != classified_objects.size(); object++) {
				double activation = 0;
				double diff = 0;
				double sum_of_squares = 0;

				activation += size_weight * abs(new_object.x_size - classified_objects[object].x_size);
				activation += size_weight * abs(new_object.y_size - classified_objects[object].y_size);
				activation += size_weight * abs(new_object.z_size - classified_objects[object].z_size);
				activation += size_weight * abs(new_object.cloud_size - classified_objects[object].cloud_size);

				for(int column = 0; column != new_object.esf_attributes.size(); column++) {
					diff = (new_object.esf_attributes[column] - classified_objects[object].esf_attributes[column]);
					sum_of_squares += std::pow(diff, 2);
				}

				activation += esf_weight * sum_of_squares;	// TO-DO: Use sum of squares or abs(diff)?
				activation_list.push_back(activation);

				double activation_sum = 0;

				for(int i = 0; i != classified_objects.size(); i++) {
					activation_sum += activation_list[i];
				}

				ROS_INFO("%f", activation_sum);
				//ROS_INFO("hello %f", activation_list[0]);

				for(int j = 0; j != classified_objects.size(); j++) {
					double normalized_activation = activation_list[j] / activation_sum;
					normalized_activation = (1.0 - normalized_activation) * 100;

					//activation_list[object] = normalized_activation;

					ROS_INFO("%f%% the object is %d", normalized_activation, j);
				}

				if(activation < lower_activation_threshold) {
					//ROS_INFO("Hello, this object is too similar to object %d", object);
					//ROS_INFO("Activation: %f", activation);

					activation_list.clear();
					return;
				}
				else if(activation >= upper_activation_threshold) {
					if(object == (classified_objects.size() - 1)) {
						classified_objects.push_back(new_object);
						ROS_INFO("hello %f", activation);

						esf_idx++;
						ROS_INFO("New object found! Currently %d objects found: ", esf_idx);
					}
				}
			}
			activation_list.clear();
		}
	}

	template <class T>
	void Classifier<T>::paramsCallback(perception::ClassifyConfig &config, uint32_t level) {
		radius_limit = config.radius_limit;
		epsilon_angle = config.epsilon_angle;
		curvature_threshold = config.curvature_threshold;
		plane_radius = config.plane_radius;
		size_weight = config.size_weight;
		esf_weight = config.esf_weight;
		lower_activation_threshold = config.lower_activation_threshold;
		upper_activation_threshold = config.upper_activation_threshold;
		object = config.object;
	}

	template <class T>
	void Classifier<T>::Callback(const perception::PointCloudArray& msg) {
		typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
		pcl::fromROSMsg(msg.cluster, *cloud);

		typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		this->computeNormals(cloud, normals);

		double size_array[4];
		this->computeSize(cloud, size_array);

		//ROS_INFO("Number of Clusters Found: %ld", msg.id);

		//ROS_INFO("Checking object: %d", msg.id);

		//ROS_INFO("Length: %f", size_array[0]);
		//ROS_INFO("Width: %f", size_array[1]);
		//ROS_INFO("Height: %f", size_array[2]);
		//ROS_INFO("Point Cloud Size: %f", size_array[3]);

		ROS_INFO("-----------------------------------------");

		DESCRIPTORS new_object;
		
		new_object.x_size = size_array[0];
		new_object.y_size = size_array[1];
		new_object.z_size = size_array[2];
		new_object.cloud_size = size_array[3];

		new_object.esf_attributes = this->ESF_Descriptors(cloud);

		this->classify(new_object);	// Main classify func

		if(msg.id == object) {
			sensor_msgs::PointCloud2 msg_out;
			pcl::toROSMsg(*cloud, msg_out);

			msg_out.header.frame_id = "velodyne";
			object_pub_.publish(msg_out);
		}

		//this->CVFH_Descriptors(cloud, normals);
		//this->GRSD_Descriptors(cloud);
		////this->ESF_Descriptors(cloud);
		//this->FPFH_Descriptors(cloud, normals);
		//this->SC3D_Descriptors(cloud, normals);
	}

/*	template <class T>
	void Classifier<T>::cloudCallback(const sensor_msgs::PointCloud2& msg) {
		typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
		pcl::fromROSMsg(msg, *cloud);

		//pcl::ExtractIndices<T> extract;
		//typename pcl::PointCloud<T>::Ptr object_cloud(new pcl::PointCloud<T>());

		//extract.setInputCloud(cloud);
		//extract.setIndices(indices);
		//extract.setNegative(false);
		//extract.filter(*object_cloud);
		
		typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		this->computeNormals(cloud, normals);

		double size_array[4];
		this->computeSize(cloud, size_array);


		ROS_INFO("Length: %f", size_array[0]);
		ROS_INFO("Width: %f", size_array[1]);
		ROS_INFO("Height: %f", size_array[2]);
		ROS_INFO("Point Cloud Size: %f", size_array[3]);


		//this->CVFH_Descriptors(cloud, normals);
		//this->GRSD_Descriptors(cloud);
		this->ESF_Descriptors(cloud);
		//this->FPFH_Descriptors(cloud, normals);
		//this->SC3D_Descriptors(cloud, normals);
	}

	template <class T>
	void Classifier<T>::indicesCallback(const perception::indices& msg) {
		if(msg.id == 0) zero_obj_found = true;

		while(zero_obj_found == false) {
			return;
		}
		
		if(msg.id == msg.len - 1) {
			indices.clear();
			zero_obj_found = false;
		}

		indices.push_back(msg.indices[msg.id]);

	}*/
}