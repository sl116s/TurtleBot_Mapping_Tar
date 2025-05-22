#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>




#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h" 
#include <cstdlib> 
#include <ctime> 

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/iss_3d.h>






class Node{
	protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastCloud;
		bool isFirstCloud = true;
		ros::Subscriber odometry; 
		ros::Subscriber laserSub; 
		Eigen::Matrix<float, 4, 4> rotation_matrix_float;
		double anterior_angel;
		double angle_circuit;
		bool capturar;

		pcl::PointCloud<pcl::PointWithScale>::Ptr cloud_keypoints_SSI_1;
		pcl::PointCloud<pcl::PointWithScale>::Ptr cloud_keypoints_SSI_2;


		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypoints_Hrris3D_1;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keypoints_Hrris3D_2;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_descriptores_fpfh_1;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_descriptores_fpfh_2;

		Eigen::Matrix4f transformation_total;
		Eigen::Matrix4f transformation_total_final;

	public:
		Node(ros::NodeHandle& nh) {
			anterior_angel = 0;
			angle_circuit = 0;
			capturar = false;

			visu_pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
			lastCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

			cloud_keypoints_SSI_1.reset(new pcl::PointCloud<pcl::PointWithScale> ());				//Declarar variable PointCloud para Extraer Caracteristicas actual i
			cloud_keypoints_SSI_2.reset(new pcl::PointCloud<pcl::PointWithScale> ());				//Declarar variable PointCloud para Extraer Caracteristicas actual i+1

			cloud_descriptores_fpfh_1.reset(new pcl::PointCloud<pcl::FPFHSignature33> ());		//Declarar variable PointCloud para Extraer Descriptores actual i
			cloud_descriptores_fpfh_2.reset(new pcl::PointCloud<pcl::FPFHSignature33> ());		//Declarar variable PointCloud para Extraer Descriptores actual i+1

			cloud_keypoints_Hrris3D_1.reset(new pcl::PointCloud<pcl::PointXYZI>());
			cloud_keypoints_Hrris3D_2.reset(new pcl::PointCloud<pcl::PointXYZI>());

			transformation_total = Eigen::Matrix4f::Identity(); //Transformacion total
			transformation_total_final = Eigen::Matrix4f::Identity(); //Transformacion total

			//srand(time(NULL));
			odometry = nh.subscribe("odom", 5, &Node::commandOdom, this); 
			//laserSub = nh.subscribe("base_scan", 1, &Node::commandCallback, this);
		};

		void simpleVis ()
		{
			pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
			//pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
			//viewer.setBackgroundColor(0.0, 0.0, 0.0); // Fondo negro
			//viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);						
			//viewer.showCloud (globalMap);
			//viewer.addPointCloud(globalMap, "cloud");
			//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud"); // Blanco
			while(!viewer.wasStopped())
			{
				/*viewer.setCameraPosition(position.x(), position.y(), position.z(), 
										forward_vec.x(), forward_vec.y(), forward_vec.z(), 
										up_vec.x(), up_vec.y(), up_vec.z());*/
				viewer.showCloud (globalMap,"cloud");
				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			}

		};

		void commandOdom(const nav_msgs::Odometry::ConstPtr& msg) {  
			//position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

			double pw = msg->pose.pose.orientation.z;

			double angle_z = 2.0 * std::acos(msg->pose.pose.orientation.w); // Ángulo en radianes
			if (angle_z < 0) {
				angle_z += 2 * M_PI;
			}
			if (angle_z >= (2 * M_PI)) {
				angle_z -= 2 * M_PI;
			}
			rotation_matrix_float << 	cos(angle_z),0, sin(angle_z),0,
										0,1.0f,0,0,
										-sin(angle_z),0, cos(angle_z),0,
										0,0,0,1.0f;	
			
			


			// Convertir de radianes a grados
			double angle_deg = angle_z * (180.0 / M_PI);

			// Ajustar el ángulo en el rango de 0 a 360 grados
			angle_deg = fmod(angle_deg, 360.0);
			if (angle_deg < 0) {
				angle_deg += 360.0;
			}

			if(anterior_angel == 0) {
				anterior_angel = angle_deg;
			} 
			else {
				double actual_angle = anterior_angel - angle_deg;
				if(actual_angle > 5 || actual_angle < -5) {
					anterior_angel = angle_deg;
					capturar = true;
				}
			}


			/*double angle_update = 0;
			if(anterior_angel == 0) {
				anterior_angel = angle_deg;
			} 
			else {
				actual_angle = anterior_angel - angle_deg;
				if(actual_angle > 10 || actual_angle < -10) {
					anterior_angel = angle_deg;
					angle_circuit += actual_angle;
					actual_angle = 0;

					if (angle_circuit < -360) {
						angle_circuit += 360.0;
					}
					if (angle_circuit > 360) {
						angle_circuit -= 360.0;
					}
					capturar = true;
				} 
			}*/

			//test = angle_deg;
			//double radians = test * M_PI / 180.0f;
			//ROS_INFO_STREAM("Odometry rotacion oz: " << angle_circuit); 	
			
			//std::cout << "Rotation Matrix (4x4) float:\n" << rotation_matrix_float << std::endl;
			/*tf2::Quaternion quat(position.x(), position.y(), position.z(), pw);
    		tf2::Matrix3x3 mat(quat);

			double roll, pitch, yaw;
    		mat.getRPY(roll, pitch, yaw);

			ROS_INFO_STREAM("Odometry rotacion oz: " << cos(yaw)); 
			rotation_matrix_float << 	cos(yaw), -sin(yaw),0,0,
										sin(yaw), cos(yaw),0,0,
										0,0,1.0f,0,
										0,0,0,1.0f;	*/

			/*tf2::Quaternion quat(position.x(), position.y(), position.z(), pw);
    		tf2::Matrix3x3 mat(quat);

			double roll, pitch, yaw;
    		mat.getRPY(roll, pitch, yaw);

			forward_vec << cos(yaw) * cos(pitch), sin(yaw) * cos(pitch), -sin(pitch);
			up_vec << cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll), 
					sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll),
					cos(pitch) * sin(roll);*/

			
			
			//transformation.translation() << position;
			/*transformation.linear().col(0) << forward_vec;
			transformation.linear().col(1) << up_vec.cross(forward_vec).normalized();
			transformation.linear().col(2) << up_vec;*/

			/*tf2::Quaternion quat(0.0, 0.0, 0.0, pw); // Solo se utiliza la orientación en Z
    		tf2::Matrix3x3 mat(quat);

			double roll, pitch, yaw;
			mat.getRPY(roll, pitch, yaw);

			// Construir la matriz de rotación solo con la orientación
			Eigen::Matrix3d rotation_matrix;
			rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
			ROS_INFO_STREAM("0,0: " << rotation_matrix(0,0)); 
			ROS_INFO_STREAM("3,3: " << rotation_matrix(2,2));
			ROS_INFO_STREAM("0,1: " << rotation_matrix(0,1));
			ROS_INFO_STREAM("0,2: " << rotation_matrix(0,2));
			ROS_INFO_STREAM("1,2: " << rotation_matrix(1,2));
			rotation_matrix_float << 	rotation_matrix(0,0), rotation_matrix(0,1),rotation_matrix(0,2),0,
										rotation_matrix(1,0), rotation_matrix(1,1),rotation_matrix(1,2),0,
										rotation_matrix(2,0),rotation_matrix(2,1),rotation_matrix(2,2),0,
										0,0,0,1.0f;*/

		};

		void concatenarPuntoNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPuntos, pcl::PointCloud<pcl::PointNormal>::Ptr& cloudNormales) {
			for(int i=0; i < cloudNormales->points.size(); i++){
				cloudNormales->points[i].x = cloudPuntos->points[i].x;
				cloudNormales->points[i].y = cloudPuntos->points[i].y;
				cloudNormales->points[i].z = cloudPuntos->points[i].z;
			}
		}

		// Metódo para calcular las normales con nubes con RGB
		void calculoNormalesKdTreeRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPuntos, pcl::PointCloud<pcl::PointNormal>::Ptr& cloudNormales){
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> estimacion;
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());
			
			estimacion.setInputCloud(cloudPuntos);
			estimacion.setSearchMethod(kdtree);
			estimacion.setRadiusSearch(1.0f);
			estimacion.compute(*cloudNormales);
		};

		void calculoKeypointsHarris3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPuntos, pcl::PointCloud<pcl::PointNormal>::Ptr& cloudNormales, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudResultado) {
			concatenarPuntoNormal(cloudPuntos, cloudNormales);
			
			pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::PointNormal>* harris = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI, pcl::PointNormal>;
			harris->setRadiusSearch(0.05f * 3);
			harris->setInputCloud(cloudPuntos);
			harris->setNormals(cloudNormales);
			harris->setNumberOfThreads(6);
			harris->compute(*cloudResultado);

		}


		void calculoKeypointsSIFT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPuntos, pcl::PointCloud<pcl::PointNormal>::Ptr& cloudNormales, pcl::PointCloud<pcl::PointWithScale>::Ptr& cloudResultado){
			concatenarPuntoNormal(cloudPuntos,cloudNormales);
			pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
			pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal> ());
			
			const float min_scale = std::stof("0.01");   	//0.01                 
			const int n_octaves = std::stof("3");      		//3                   
			const int n_scales_per_octave = std::stof("4");	//4
			const float min_contrast = std::stof("0.001"); 	//0.001

			sift.setSearchMethod(kdtree);
			sift.setScales(min_scale,n_octaves,n_scales_per_octave);
			sift.setMinimumContrast(min_contrast);
			sift.setInputCloud(cloudNormales);

			sift.compute(*cloudResultado);
		};

		void calculoDescriptoresFPFH_para_harris(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPuntos, pcl::PointCloud<pcl::PointXYZI>::Ptr& keypoint, pcl::PointCloud<pcl::PointNormal>::Ptr& cloudNormales, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors) {
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud (*keypoint, *keypoints_xyzrgb);

			// Objeto de estimación de descriptores FPFH
			pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;


			fpfh.setInputCloud(keypoints_xyzrgb);
			fpfh.setInputNormals(cloudNormales);
			fpfh.setSearchSurface(cloudPuntos);
			
			fpfh.setSearchMethod(kdtree);
			fpfh.setRadiusSearch(0.05);
			fpfh.compute(*descriptors);
		}

		void calculoDescriptoresFPFHRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPuntos, pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints, pcl::PointCloud<pcl::PointNormal>::Ptr& cloudNormales, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors) {
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

			// Objeto de estimación de descriptores FPFH
			pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;


			fpfh.setInputCloud(keypoints_xyzrgb);
			fpfh.setInputNormals(cloudNormales);
			fpfh.setSearchSurface(cloudPuntos);
			
			fpfh.setSearchMethod(kdtree);
			fpfh.setRadiusSearch(0.05);
			fpfh.compute(*descriptors);
		};

		// Emparejamientos de los descriptores
		void encontrarEmparejamientosFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors_1, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors_2, pcl::Correspondences& correspondencias){
			pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr;
			
			corr.setInputSource(descriptors_1);
			corr.setInputTarget(descriptors_2);
			corr.determineReciprocalCorrespondences (correspondencias);
		}

		Eigen::Matrix4f get_Mejor_Transformacion_RANSAC_With_Scale(pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_1, pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_2, pcl::Correspondences& correspondencias, pcl::Correspondences& correspondencias_no_valida) {
			// RANSAC
			//pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> svd;
			pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> mejor;
			//mejor.setMinSampleDistance(0.05f * 3);
			//mejor.setMaxCorrespondenceDistance(0.1);
			mejor.setInlierThreshold(0.05f * 3);
			mejor.setMaximumIterations(100000);
			mejor.setRefineModel(true);

			mejor.setInputSource(keypoints_1);
			mejor.setInputTarget(keypoints_2);

			pcl::CorrespondencesConstPtr cor(new pcl::Correspondences(correspondencias));
			mejor.setInputCorrespondences(cor); 
			mejor.getCorrespondences(correspondencias_no_valida);
			//mejor.setTransformationEstimation(svd);

			return mejor.getBestTransformation();
		}

		Eigen::Matrix4f get_Mejor_Transformacion_RANSAC_With_CloudPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_1, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_2, pcl::Correspondences& correspondencias, pcl::Correspondences& correspondencias_no_valida) {
			// RANSAC
			//pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> svd;
			pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> mejor;
			//mejor.setMinSampleDistance(0.05f * 3);
			//mejor.setMaxCorrespondenceDistance(0.1);
			mejor.setInlierThreshold(0.05f * 3);
			mejor.setMaximumIterations(100000);
			mejor.setRefineModel(true);

			mejor.setInputSource(keypoints_1);
			mejor.setInputTarget(keypoints_2);

			pcl::CorrespondencesConstPtr cor(new pcl::Correspondences(correspondencias));
			mejor.setInputCorrespondences(cor); 
			mejor.getCorrespondences(correspondencias_no_valida);
			//mejor.setTransformationEstimation(svd);

			return mejor.getBestTransformation();
		}

		Eigen::Matrix4f matchingICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPuntos_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPuntos_2) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
			//icp.setMaxCorrespondenceDistance(1000.0);
			//icp.setTransformationEpsilon(0.05f * 3);
			icp.setMaximumIterations(100000);

			cout << "cloud_1: " << cloudPuntos_1->size() << endl;
			cout << "cloud_2: " << cloudPuntos_2->size() << endl;

			icp.setInputSource(cloudPuntos_1);
			icp.setInputTarget(cloudPuntos_2);
			icp.getFinalTransformation();
			//icp.align(*output);

			//cout << "output: " << output->size() << endl;

			return icp.getFinalTransformation();
		}

		void update_position(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered) {
			for (size_t i = 0; i < cloud_filtered->size(); ++i) {
				pcl::PointXYZRGB& point = cloud_filtered->points[i];

				// Crear un vector homogéneo con las coordenadas xyz y 1
				Eigen::Vector4f point_vector(point.x, point.y, point.z, 1.0f);

				// Multiplicar el vector por la matriz de rotación
				Eigen::Vector4f transformed_point_vector = rotation_matrix_float * point_vector;

				// Actualizar las coordenadas del punto con las nuevas coordenadas transformadas
				point.x = transformed_point_vector(0);
				point.y = transformed_point_vector(1);
				point.z = transformed_point_vector(2);
			}
		}

		void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normales (new pcl::PointCloud<pcl::PointNormal>);
			//pcl::PointCloud<pcl::PointXYZI>::Ptr aux_harris (new pcl::PointCloud<pcl::PointXYZI>);
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_resultado (new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
			vGrid.setInputCloud (cloud);
			vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
			vGrid.filter (*cloud_filtered);



			if(isFirstCloud) {
				update_position(cloud_filtered);

				calculoNormalesKdTreeRGB(cloud_filtered, cloud_normales);

				cout << "Puntos antes de extraer Caracteristicas (Primera iteracion): " << cloud_filtered->size() << endl;
				calculoKeypointsSIFT(cloud_filtered, cloud_normales,cloud_keypoints_SSI_1);
				//calculoKeypointsHarris3D(cloud_filtered, cloud_normales,cloud_keypoints_Hrris3D_1);
				cout << "Puntos despues de extraer Caracteristicas (Primera iteracion): " << cloud_keypoints_SSI_1->size() << endl;

				calculoDescriptoresFPFHRGB(cloud_filtered, cloud_keypoints_SSI_1, cloud_normales, cloud_descriptores_fpfh_1);
				//calculoDescriptoresFPFH_para_harris(cloud_filtered, cloud_keypoints_Hrris3D_1, cloud_normales, cloud_descriptores_fpfh_1);

				*globalMap += *cloud_filtered;
				lastCloud = cloud_filtered;
				isFirstCloud = false;
			}
			else {
				if(capturar) {
					update_position(cloud_filtered);
					calculoNormalesKdTreeRGB(cloud_filtered, cloud_normales);

					cout << "Puntos antes de extraer Caracteristicas: " << cloud_filtered->size() << endl;
					calculoKeypointsSIFT(cloud_filtered, cloud_normales,cloud_keypoints_SSI_2);
					//calculoKeypointsHarris3D(cloud_filtered, cloud_normales,cloud_keypoints_Hrris3D_2);
					cout << "Puntos despues de extraer Caracteristicas: " << cloud_keypoints_SSI_2->size() << endl;

					calculoDescriptoresFPFHRGB(cloud_filtered, cloud_keypoints_SSI_2, cloud_normales, cloud_descriptores_fpfh_2);
					//calculoDescriptoresFPFH_para_harris(cloud_filtered, cloud_keypoints_Hrris3D_2, cloud_normales, cloud_descriptores_fpfh_2);

					pcl::Correspondences correspondencias;
					pcl::Correspondences correspondencias_no_validas;
					encontrarEmparejamientosFPFH(cloud_descriptores_fpfh_2, cloud_descriptores_fpfh_1, correspondencias);

					cout << "Correspondencias size: " << correspondencias.size() << endl;
					Eigen::Matrix4f transformation = get_Mejor_Transformacion_RANSAC_With_Scale(cloud_keypoints_SSI_2,cloud_keypoints_SSI_1,correspondencias,correspondencias_no_validas);
					//Eigen::Matrix4f transformation = get_Mejor_Transformacion_RANSAC_With_CloudPoint(cloud_keypoints_Hrris3D_2,cloud_keypoints_Hrris3D_1,correspondencias,correspondencias_no_validas);
					cout << "Correspondencias no validas size: " << correspondencias_no_validas.size() << endl;

					transformation_total *= transformation;

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformada (new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::transformPointCloud(*cloud_filtered, *cloud_transformada, transformation_total);


					Eigen::Matrix4f transformation_final = matchingICP(cloud_transformada, lastCloud);
					cout << "Terminado la estemacion."  << endl;

					transformation_total_final *= transformation_final;

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformada_final (new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::transformPointCloud(*cloud_transformada, *cloud_transformada_final, transformation_total_final);

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_aux (new pcl::PointCloud<pcl::PointXYZRGB>);
					*map_aux = *globalMap;
					*map_aux += *cloud_transformada_final;

					pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
					vGrid.setInputCloud (map_aux);
					vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
					vGrid.filter (*globalMap);

					*cloud_keypoints_SSI_1 = *cloud_keypoints_SSI_2;
					//*cloud_keypoints_Hrris3D_1 = *cloud_keypoints_Hrris3D_2;

					*cloud_descriptores_fpfh_1 = *cloud_descriptores_fpfh_2;

					*lastCloud = *cloud_filtered;
					capturar = false;
					cout << "Done."  << endl;
				}
			}
		};


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	Node node(nh);


	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, &Node::callback, &node);

	boost::thread t(&Node::simpleVis, &node);

	while(ros::ok())
	{
		ros::spinOnce();
	}

}
