#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <upd.h>
#include <export.h>
#include <math.h>
#include <stdlib.h>
#include <string> 
#include <fstream>
#include <vector>
#include <ctime>
using namespace std;

// Constant definitions
#define _USE_MATH_DEFINES
#define PI 3.14159265

// Common Point Cloud type used in this processing
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;


pcl::visualization::CloudViewer viewer2 ("Show Points");

// UPD published data
PointCloud::Ptr upd_data;

/*
* This function classify if the laser is pointing to a location that robot is 
* not able to pass, based on upd cloud. It will publish a ROS publisher with
* a string "safe" or "unsafe" related to the condition passage, considering 
* the uneveness and robot climb capabilities that were already defined on upd 
* generation data
*/
void classify_passage_condition(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>> cloud, PointCloud::Ptr laser){

    // srand (time (NULL));

    // 1. Defining the kdtree and the point object for the search
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    pcl::PointXYZRGBA searchPoint;

    // 2. Load the UPD cloud to kdtree object 
    kdtree.setInputCloud (cloud);

    // 3. Iterate the laser points and search them using kdtree
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;

    // Good point for passage condition counter
    float greenCount = 0;
    float total = 0;

    // 4. Search the points
    for( it= laser->begin(); it!= laser->end(); it++){

      // Note: Changing the axis to be compatible with kinect axis data
      searchPoint.x = it->z; // z to x
      searchPoint.y = it->y;
      searchPoint.z = -it->x; // x to z

      // K nearest neighbor search

      int K = 10;

      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);

      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        total += pointIdxNKNSearch.size ();
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)

          if(cloud->points[ pointIdxNKNSearch[i]].g==255){
            greenCount++;
          }

      }

    }

    // 5. Determine the passage condition
    std_msgs::String msg;

    if((greenCount / total) >= std::stof (getenv("MINIMUM_GREEN_POINTS"))){
      msg.data = "safe";

    }else{
      msg.data = "unsafe";
    }

    // 6. Publish the passage condition
    ros::NodeHandle nhpass;
    ros::Publisher pub = nhpass.advertise<std_msgs::String> ("passage_condition", 1);
    pub.publish (msg);
    ros::spinOnce();
}

/*
* This function will process the callback and request the passage condition classification
* of the laser data that was converted to point cloud data
*/
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
    PointCloud::Ptr laser_data = boost::const_pointer_cast<PointCloud>(msg);

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
    pcl::transformPointCloud(*laser_data, *laser_data, translationMatrix);


    // 4. Rotation matrix definition for laser pan & tilt
    Eigen::Matrix<float, 4, 4> tiltRotationMatrix;
  
    // Z (Tilt)
    tiltRotationMatrix(0,0) = cos(-1 * std::stof (getenv("LASER_TILT")));
    tiltRotationMatrix(0,1) = -sin(-1 * std::stof (getenv("LASER_TILT")));
    tiltRotationMatrix(0,2) = 0;
    tiltRotationMatrix(0,3) = 0;

    tiltRotationMatrix(1,0) = sin(-1 * std::stof (getenv("LASER_TILT")));
    tiltRotationMatrix(1,1) = cos(-1 * std::stof (getenv("LASER_TILT")));
    tiltRotationMatrix(1,2) = 0;
    tiltRotationMatrix(1,3) = 0;

    tiltRotationMatrix(2,0) = 0;
    tiltRotationMatrix(2,1) = 0;
    tiltRotationMatrix(2,2) = 1;
    tiltRotationMatrix(2,3) = 0;  

    tiltRotationMatrix(3,0) = 0;
    tiltRotationMatrix(3,1) = 0;
    tiltRotationMatrix(3,2) = 0;
    tiltRotationMatrix(3,3) = 1;


    // Tilt Transformation
    pcl::transformPointCloud(*laser_data, *laser_data, tiltRotationMatrix);


    // Y (Pan)
    Eigen::Matrix<float, 4, 4> panRotationMatrix;

    panRotationMatrix(0,0) = cos(std::stof (getenv("LASER_PAN")));
    panRotationMatrix(0,1) = 0;
    panRotationMatrix(0,2) = sin(std::stof (getenv("LASER_PAN")));
    panRotationMatrix(0,3) = 0;

    panRotationMatrix(1,0) = 0;
    panRotationMatrix(1,1) = 1;
    panRotationMatrix(1,2) = 0;
    panRotationMatrix(1,3) = 0;

    panRotationMatrix(2,0) = -sin(std::stof (getenv("LASER_PAN")));
    panRotationMatrix(2,1) = 0;
    panRotationMatrix(2,2) = cos(std::stof (getenv("LASER_PAN")));;
    panRotationMatrix(2,3) = 0;  

    panRotationMatrix(3,0) = 0;
    panRotationMatrix(3,1) = 0;
    panRotationMatrix(3,2) = 0;
    panRotationMatrix(3,3) = 1;

    // Pan Transformation
    pcl::transformPointCloud(*laser_data, *laser_data, panRotationMatrix);

    // 4. Classify the passage condition
    // classify_passage_condition(upd_data, laser_data);


    // 4. Include laser data into kinect UPD processed data
    pcl::PointXYZRGBA point2;
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator it2;

    for( it2= laser_data->begin(); it2!= laser_data->end(); it2++){
        point2.x = it2->z;
        point2.y = it2->y;
        point2.z = -it2->x;
        point2.r = 0;
        point2.g = 0;
        point2.b = 254;
        point2.a = 255;
        upd_data->push_back(point2);
    }
  // printf("- Laser call back -");
  if (!viewer2.wasStopped()){
      viewer2.showCloud (upd_data);
  }

    // // 5. Reseting the data
    // upd_result.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
}

/*
* This function will process the callback of UPD publisher and subscribe to the laser publisher
* that will request the passage classification next
*/
void upd_callback(const PointCloud::ConstPtr& msg){
 
  upd_data = boost::const_pointer_cast<PointCloud>(msg);
  
  // 8. Processing laser
  ros::NodeHandle nhk;
  ros::Subscriber sub = nhk.subscribe<PointCloud>("/laser/point_cloud", 1, laser_callback);
  ros::Rate loop_rate(50);
  loop_rate.sleep();
  ros::spin();

}


/*
* Main function that starts the process
*/
int main(int argc, char** argv){
   
  // 0. Printing the parameters
  printf("===================================================");
  printf("Info: Setup parameters \n");
  printf("LASER_TO_KINECT_X = %s \n", getenv("LASER_TO_KINECT_X"));
  printf("LASER_TO_KINECT_Y = %s \n", getenv("LASER_TO_KINECT_Y"));
  printf("LASER_TO_KINECT_Z = %s \n", getenv("LASER_TO_KINECT_Z"));
  printf("MINIMUM_GREEN_POINTS = %s \n", getenv("MINIMUM_GREEN_POINTS"));
  printf("=================================================== \n");

  ros::init(argc, argv, "passage_classifier_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("upd_point_cloud_classification", 1, upd_callback);
  ros::Rate loop_rate(1000);
  loop_rate.sleep();
  printf("Info: 4. Publishing Passage Classified Data \n");
  ros::spin();
}