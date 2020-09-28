#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/compression/octree_pointcloud_compression.h>
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
#include <limits>
using namespace std;

// Constant definitions
#define _USE_MATH_DEFINES
#define PI 3.14159265
typedef std::numeric_limits< double > dbl;

// Common Point Cloud type used in this processing
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;


// pcl::visualization::CloudViewer viewer2 ("Show Points");

class PassageClassificationProcess{
    public:

        bool upd_data_ready = false;
        bool laser_data_ready = false;
        bool laser_data_orientation_ready = false;

        PassageClassificationProcess(){
            // 1. Subscribe to the UPD Node
            sub_upd_ = nh_.subscribe<PointCloud>("upd_point_cloud_classification", 1, &PassageClassificationProcess::upd_callback, this);

            // 2. Subscribe to the Laser data Node
            sub_laser_data_ = nh_.subscribe<PointCloud>("/laser/point_cloud", 1, &PassageClassificationProcess::laser_data_callback, this);

            // 3. Subscribe to the Laser orientation data Node
            sub_laser_orientation_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &PassageClassificationProcess::laser_data_orientation_callback, this);

            // 4. Publish UPD Rviz data
            pub_passage_condition_ = nh_.advertise<std_msgs::String> ("passage_condition", 1);
        }
    
        /*
        * This function will process the callback of UPD publisher and subscribe to the laser publisher
        * that will request the passage classification next
        */
        void upd_callback(const PointCloud::ConstPtr& msg){

          PassageClassificationProcess::upd_data = boost::const_pointer_cast<PointCloud>(msg);

          PassageClassificationProcess::upd_data_ready = true;

          printf("upd_callback \n");
        }

        /*
        * This function will process the callback and request the passage condition classification
        * of the laser data that was converted to point cloud data
        */
        void laser_data_callback(const PointCloud::ConstPtr& msg){


            // 0. Reset laser data processing status
            // PassageClassificationProcess::laser_data_ready = false;

            printf("***** Laser Run - start ******\n");
            // 1. Casting the laser point cloud object
            PointCloud::Ptr laser_data_raw = boost::const_pointer_cast<PointCloud>(msg);

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
          
            pcl::transformPointCloud(*laser_data_raw, *laser_data_raw, translationMatrix);

            // 4. Rotation matrix definition for laser pan & tilt
            Eigen::Matrix<float, 4, 4> tiltRotationMatrix;

            // Getting laser pan tilt
            double tilt = PassageClassificationProcess::laser_tilt;
            double pan = PassageClassificationProcess::laser_pan;

            // Z (Tilt)
            tiltRotationMatrix(0,0) = cos(-1 * tilt);
            tiltRotationMatrix(0,1) = -sin(-1 * tilt);
            tiltRotationMatrix(0,2) = 0;
            tiltRotationMatrix(0,3) = 0;

            tiltRotationMatrix(1,0) = sin(-1 * tilt);
            tiltRotationMatrix(1,1) = cos(-1 * tilt);
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
            pcl::transformPointCloud(*laser_data_raw, *laser_data_raw, tiltRotationMatrix);
            
            // Y (Pan)
            Eigen::Matrix<float, 4, 4> panRotationMatrix;

            panRotationMatrix(0,0) = cos(pan);
            panRotationMatrix(0,1) = 0;
            panRotationMatrix(0,2) = sin(pan);
            panRotationMatrix(0,3) = 0;

            panRotationMatrix(1,0) = 0;
            panRotationMatrix(1,1) = 1;
            panRotationMatrix(1,2) = 0;
            panRotationMatrix(1,3) = 0;

            panRotationMatrix(2,0) = -sin(pan);
            panRotationMatrix(2,1) = 0;
            panRotationMatrix(2,2) = cos(pan);;
            panRotationMatrix(2,3) = 0;  

            panRotationMatrix(3,0) = 0;
            panRotationMatrix(3,1) = 0;
            panRotationMatrix(3,2) = 0;
            panRotationMatrix(3,3) = 1;

            // Pan Transformation
            pcl::transformPointCloud(*laser_data_raw, *laser_data_raw, panRotationMatrix);


            // 3.6 Vox grid reduction
            pcl::VoxelGrid<pcl::PointXYZRGBA> sor; 
            sor.setInputCloud (laser_data_raw);
            sor.setLeafSize (std::stof(getenv("VOX_GRID_LEAF_X")), std::stof(getenv("VOX_GRID_LEAF_Y")), std::stof(getenv("VOX_GRID_LEAF_Z")));
            PointCloud::Ptr laser_data_filtered (new PointCloud);
            sor.filter (*laser_data_filtered);

            PassageClassificationProcess::laser_data = laser_data_filtered;
            
            // Set laser data process to ready
            PassageClassificationProcess::laser_data_ready = true;

            cout.precision(dbl::max_digits10);
            cout << "Pan: " << pan << endl;
            cout << "Tilt: " << tilt << endl;
            printf("***** Laser Run - finish ******\n");

            // 4. Include laser data into kinect UPD processed data
            // if( PassageClassificationProcess::upd_data_ready==true){
            //     // PointCloud::Ptr upd_temp = PassageClassificationProcess::upd_data;
            //     // pcl::PointXYZRGBA point2;
            //     // pcl::PointCloud<pcl::PointXYZRGBA>::iterator it2;

            //     // for( it2= laser_data_filtered->begin(); it2!= laser_data_filtered->end(); it2++){
            //     //     point2.x = it2->z;
            //     //     point2.y = it2->y;
            //     //     point2.z = -it2->x;
            //     //     point2.r = 0;
            //     //     point2.g = 0;
            //     //     point2.b = 254;
            //     //     point2.a = 255;
            //     //     upd_temp->push_back(point2);
            //     // }

            //     if (!viewer2.wasStopped()){
            //         viewer2.showCloud (PassageClassificationProcess::upd_data);
            //     }
            // }
            printf("laser_callback \n");

        }

        /*
        * This function classify if the laser is pointing to a location that robot is 
        * not able to pass, based on upd cloud. It will publish a ROS publisher with
        * a string "safe" or "unsafe" related to the condition passage, considering 
        * the uneveness and robot climb capabilities that were already defined on upd 
        * generation data
        */
        void classify_passage_condition(){

            // 1. Defining the kdtree and the point object for the search
            pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
            pcl::PointXYZRGBA searchPoint;

            // 2. Load the UPD cloud to kdtree object 
            const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>> cloud = PassageClassificationProcess::upd_data;
            kdtree.setInputCloud (cloud);

            PointCloud::Ptr laser = PassageClassificationProcess::laser_data;

            // 3. Iterate the laser points and search them using kdtree
            pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;

            // Good point for passage condition counter
            float greenCount = 0;
            float notGreenCount = 0;
            float total = 0;

            // 4. Search the points
            for( it= laser->begin(); it!= laser->end(); it++){

              // Note: Changing the axis to be compatible with kinect axis data
              searchPoint.x = it->z; // z to x
              searchPoint.y = it->y;
              searchPoint.z = -it->x; // x to z
                    cout.precision(dbl::max_digits10);
                    cout << "z: " << searchPoint.z << endl;
                    cout << "x: " << searchPoint.x << endl;
                    cout << "y: " << searchPoint.y << endl;
              // K nearest neighbor search

              int K = 10;

              std::vector<int> pointIdxNKNSearch(K);
              std::vector<float> pointNKNSquaredDistance(K);

              if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
              {
                
                for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                  
                  total++;
                  // 
                  if(cloud->points[ pointIdxNKNSearch[i]].g==255
                     && searchPoint.y <= std::stof (getenv("HEIGHT_THRESHOLD"))
                  ){
                    greenCount++;
                  }
                }
              }

            }

            // 5. Determine the passage condition
            std_msgs::String msg;

            if((greenCount / total) >= std::stof (getenv("MINIMUM_GREEN_POINTS"))){
              msg.data = "safe";
              cout.precision(dbl::max_digits10);
              cout << "safe total: " << total << endl;
              cout << "safe greenCount: " << greenCount << endl;
              

            }else{
              msg.data = "unsafe";
              cout.precision(dbl::max_digits10);
              cout << "unsafe total: " << total << endl;
              cout << "unsafe greenCount: " << greenCount << endl;
              
            }

            // 6. Publish the passage condition
           pub_passage_condition_.publish (msg);

           printf("classify_passage_condition \n");
           printf("***** Finished ******* \n");
        }


        /*
        * This function will process the callback for laser orientation
        */
        void laser_data_orientation_callback(const sensor_msgs::JointStateConstPtr& msg){

          PassageClassificationProcess::laser_tilt = msg->position[0];
          PassageClassificationProcess::laser_pan = msg->position[1];

        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_upd_;
        ros::Subscriber sub_laser_data_;
        ros::Subscriber sub_laser_orientation_;
        ros::Publisher pub_passage_condition_;
        PointCloud::Ptr upd_data;
        PointCloud::Ptr laser_data;
        double laser_pan = 0.0;
        double laser_tilt = 0.0;
  
};

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
  printf("HEIGHT_THRESHOLD = %s \n", getenv("HEIGHT_THRESHOLD"));
  printf("ROS_LOOP_RATE = %s \n", getenv("ROS_LOOP_RATE"));
  printf("=================================================== \n");

    // 1. ROS Init
    printf("Info: 1. ROS Init \n");
    ros::init (argc, argv, "passage_classifier_node");

    // 2. Publishing UPD data 
    printf("Info: 2. Publishing Passage Classification data \n");
    PassageClassificationProcess pcProcess;

    // 3. Defining ROS time cycle in HERTZ
    ros::Rate loop_rate(std::stod (getenv("ROS_LOOP_RATE")));

    // 4. ROS loop
    while (ros::ok()) {
      // pcProcess.laser_data_orientation_ready ||
      // std::cout << std::boolalpha;   
      // std::cout<<pcProcess.laser_data_ready<<"\n"; 
      // std::cout<<pcProcess.upd_data_ready<<"\n"; 
      if(pcProcess.laser_data_ready == true && pcProcess.upd_data_ready == true){
          pcProcess.classify_passage_condition();
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
}