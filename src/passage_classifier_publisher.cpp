#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <upd.h>
#include <export.h>
#include <math.h>
#include <stdlib.h>
#include <string> 
#include <fstream>
using namespace std;

// Constant definitions
#define _USE_MATH_DEFINES
#define PI 3.14159265

// Common Point Cloud type used in this processing
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
pcl::visualization::CloudViewer viewer2 ("Show Points");

// UPD published data
PointCloud::Ptr upd_data;

void laser_callback(const PointCloud::ConstPtr& msg){
    // printf ("Cloud: width = %d, height = %d, sensor_origin = %d\n", msg->width, msg->height, msg->sensor_origin_);
    // printf("**** Laser sensor orientation **** ");
    // Eigen::Quaternionf myQuaternion = msg->sensor_orientation_; //The Quaternion to print

    // std::cout << "Debug: " << "sensor_orientation_.w() = " << myQuaternion.w() << std::endl; //Print out the scalar
    // std::cout << "Debug: " << "sensor_orientation_.vec() = " << myQuaternion.vec() << std::endl; //Print out the orientation vector
    
    // Eigen::Quaternionf myQuaternion2 = msg->sensor_orientation_; //The Quaternion to print

    // std::cout << "Debug: " << "sensor_origin_.w() = " << myQuaternion2.w() << std::endl; //Print out the scalar
    // std::cout << "Debug: " << "sensor_origin_.vec() = " << myQuaternion2.vec() << std::endl; //Print out the orientation vector

    // BOOST_FOREACH (const pcl::PointXYZRGBA& pt, msg->points)
    // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    // upd_result->sensor_orientation_ = Eigen::Quaternionf (0, -1, 0, 0);

    // Rotation
    // Eigen::Matrix4f eRot;
    // Eigen::Quaternionf PCRot;
    // Eigen::Vector3f PCTrans;

    // 1. Casting the laser point cloud object
    PointCloud::Ptr msg2 = boost::const_pointer_cast<PointCloud>(msg);

    // 2. Translation matrix definition for laser 
    // point frame to kinect frame
    Eigen::Matrix<float, 4, 4> translationMatrix;

    translationMatrix(0,0) = 1;
    translationMatrix(0,1) = 0;
    translationMatrix(0,2) = 0;
    translationMatrix(0,3) = std::stof (getenv("LASER_TO_KINECT_X"));

    translationMatrix(1,0) = 0;
    translationMatrix(1,1) = 1;
    translationMatrix(1,2) = 0;
    translationMatrix(1,3) = std::stof (getenv("LASER_TO_KINECT_Y"));;

    translationMatrix(2,0) = 0;
    translationMatrix(2,1) = 0;
    translationMatrix(2,2) = 1;
    translationMatrix(2,3) = std::stof (getenv("LASER_TO_KINECT_Z"));;    

    translationMatrix(3,0) = 0;
    translationMatrix(3,1) = 0;
    translationMatrix(3,2) = 0;
    translationMatrix(3,3) = 1;
   
    // 3. Translation the laser points
    pcl::transformPointCloud(*msg2, *msg2, translationMatrix);

    // 4. Include laser data into kinect UPD processed data
    pcl::PointXYZRGBA point;
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;

    for( it= msg2->begin(); it!= msg2->end(); it++){
        point.x = it->z;
        point.y = it->y;
        point.z = -it->x;
        point.r = 0;
        point.g = 0;
        point.b = 254;
        point.a = 255;
        upd_data->push_back(point);
    }
  printf("- Laser call back -");
  if (!viewer2.wasStopped()){
      viewer2.showCloud (upd_data);
  }

    // // 5. Reseting the data
    // upd_result.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
}

void upd_callback(const PointCloud::ConstPtr& msg){
 
  upd_data = boost::const_pointer_cast<PointCloud>(msg);
  
  // 8. Processing laser
  ros::NodeHandle nhk;
  ros::Subscriber sub = nhk.subscribe<PointCloud>("/laser/point_cloud", 1, laser_callback);
  ros::Rate loop_rate(50);
  loop_rate.sleep();
  ros::spin();

}

int main(int argc, char** argv){
   
  // 0. Printing the parameters
  printf("===================================================");
  printf("Info: Setup parameters \n");
  printf("LASER_TO_KINECT_X = %s \n", getenv("LASER_TO_KINECT_X"));
  printf("LASER_TO_KINECT_Y = %s \n", getenv("LASER_TO_KINECT_Y"));
  printf("LASER_TO_KINECT_Z = %s \n", getenv("LASER_TO_KINECT_Z"));
  printf("=================================================== \n");

  ros::init(argc, argv, "passage_classifier_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("upd_point_cloud_classification", 1, upd_callback);
  ros::Rate loop_rate(50);
  loop_rate.sleep();
  printf("Info: 4. Publishing Passage Classified Data \n");
  ros::spin();
}