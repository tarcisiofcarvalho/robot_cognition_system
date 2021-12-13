#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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
#include <sstream>
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
        bool classifier_ready = true;

        PassageClassificationProcess(){
            // 1. Subscribe to the UPD Node
            sub_upd_ = nh_.subscribe<PointCloud>("upd_point_cloud_classification", 1, &PassageClassificationProcess::upd_callback, this);

            // 2. Subscribe to the Laser data Node
            sub_laser_data_ = nh_.subscribe<PointCloud>("/laser/point_cloud", 1, &PassageClassificationProcess::laser_data_callback, this);

            // 3. Subscribe to the Laser orientation data Node
            sub_laser_orientation_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &PassageClassificationProcess::laser_data_orientation_callback, this);
            sub_laser_sim_pan_data_ = nh_.subscribe<std_msgs::Float64>("/laser_sim_pan", 1, &PassageClassificationProcess::laser_sim_pan_data_orientation_callback, this);
            sub_laser_sim_tilt_data_ = nh_.subscribe<std_msgs::Float64>("/laser_sim_tilt", 1, &PassageClassificationProcess::laser_sim_tilt_data_orientation_callback, this);

            // 4. Publish UPD Rviz data
            pub_passage_condition_ = nh_.advertise<std_msgs::String> ("passage_condition", 1);

            // 5. Publish target path to RVIZ data
            target_path_ = nh_.advertise<PointCloud> ("target_path", 1);

            // 6. Publish laser_simulated_ray to RVIZ data
            laser_simulated_ray_ = nh_.advertise<PointCloud> ("laser_simulated_ray", 1);

            // 7. Publish target distance em meters
            pub_target_distance_ = nh_.advertise<std_msgs::Float64> ("target_distance", 1);

            // 8. Publish Laser Intersection in the point cloud
            pub_sim_laser_intersection_ = nh_.advertise<PointCloud> ("laser_simulated_intersection", 1);            
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

            // TFC cout.precision(dbl::max_digits10);
            // TFC cout << "Pan: " << pan << endl;
            // TFC cout << "Tilt: " << tilt << endl;
            // TFC printf("***** Laser Run - finish ******\n");

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


        PointCloud::Ptr laser_data_transform(const PointCloud::ConstPtr msg){


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

            // // 4. Rotation matrix definition for laser pan & tilt
            // Eigen::Matrix<float, 4, 4> tiltRotationMatrix;

            // // Getting laser pan tilt
            // double tilt = PassageClassificationProcess::laser_sim_tilt;
            // double pan = PassageClassificationProcess::laser_sim_pan;

            // // Z (Tilt)
            // tiltRotationMatrix(0,0) = cos(-1 * tilt);
            // tiltRotationMatrix(0,1) = -sin(-1 * tilt);
            // tiltRotationMatrix(0,2) = 0;
            // tiltRotationMatrix(0,3) = 0;

            // tiltRotationMatrix(1,0) = sin(-1 * tilt);
            // tiltRotationMatrix(1,1) = cos(-1 * tilt);
            // tiltRotationMatrix(1,2) = 0;
            // tiltRotationMatrix(1,3) = 0;

            // tiltRotationMatrix(2,0) = 0;
            // tiltRotationMatrix(2,1) = 0;
            // tiltRotationMatrix(2,2) = 1;
            // tiltRotationMatrix(2,3) = 0;  

            // tiltRotationMatrix(3,0) = 0;
            // tiltRotationMatrix(3,1) = 0;
            // tiltRotationMatrix(3,2) = 0;
            // tiltRotationMatrix(3,3) = 1;

            // // Tilt Transformation
            // pcl::transformPointCloud(*laser_data_raw, *laser_data_raw, tiltRotationMatrix);
            
            // // Y (Pan)
            // Eigen::Matrix<float, 4, 4> panRotationMatrix;

            // panRotationMatrix(0,0) = cos(pan);
            // panRotationMatrix(0,1) = 0;
            // panRotationMatrix(0,2) = sin(pan);
            // panRotationMatrix(0,3) = 0;

            // panRotationMatrix(1,0) = 0;
            // panRotationMatrix(1,1) = 1;
            // panRotationMatrix(1,2) = 0;
            // panRotationMatrix(1,3) = 0;

            // panRotationMatrix(2,0) = -sin(pan);
            // panRotationMatrix(2,1) = 0;
            // panRotationMatrix(2,2) = cos(pan);;
            // panRotationMatrix(2,3) = 0;  

            // panRotationMatrix(3,0) = 0;
            // panRotationMatrix(3,1) = 0;
            // panRotationMatrix(3,2) = 0;
            // panRotationMatrix(3,3) = 1;

            // // Pan Transformation
            // pcl::transformPointCloud(*laser_data_raw, *laser_data_raw, panRotationMatrix);


            // 3.6 Vox grid reduction
            pcl::VoxelGrid<pcl::PointXYZRGBA> sor; 
            sor.setInputCloud (laser_data_raw);
            sor.setLeafSize (std::stof(getenv("VOX_GRID_LEAF_X")), std::stof(getenv("VOX_GRID_LEAF_Y")), std::stof(getenv("VOX_GRID_LEAF_Z")));
            PointCloud::Ptr laser_data_filtered (new PointCloud);
            sor.filter (*laser_data_filtered);

            // Set laser data process to ready
            PassageClassificationProcess::laser_data_ready = true;
            printf("laser_callback \n");
            return laser_data_filtered;
            


            // TFC cout.precision(dbl::max_digits10);
            // TFC cout << "Pan: " << pan << endl;
            // TFC cout << "Tilt: " << tilt << endl;
            // TFC printf("***** Laser Run - finish ******\n");

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

            // 3. Collect Laser data
            PointCloud::Ptr laser;
            if(std::string (getenv("LASER_TYPE")) == "simulated"){
              // 3.1 Simulated laser from Pan and Tilt data
              laser = generate_virtual_ray_laser_data_in_point_cloud(PassageClassificationProcess::laser_sim_pan, 
                                                               PassageClassificationProcess::laser_sim_tilt,
                                                               std::stod (getenv("LASER_X")), 
                                                               std::stod (getenv("LASER_Y")), 
                                                               std::stod (getenv("LASER_Z")));
                                                              
                  sensor_msgs::PointCloud2 msgcloud;
                  pcl::toROSMsg(*laser, msgcloud); 
                  std::string tf_frame;
                  tf_frame = "base_link";
                  msgcloud.header.frame_id = tf_frame;
                  msgcloud.header.stamp = ros::Time::now();
                  laser_simulated_ray_.publish (msgcloud);
            }else{
              // 3.2 Real laser
              laser = PassageClassificationProcess::laser_data;
            }
            

            // 3. Iterate the laser points and search them using kdtree
            pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;

            // Good point for passage condition counter
            float greenCount = 0;
            float redCount = 0;
            float notGreenCount = 0;
            double targetX = 0;
            double targetY = 0;
            double targetZ = 0;
            float tempGreenCount = 0;
            float tempRedCount = 0;
            float distanceCount = INFINITY;
            float tempDistanceCount = 0;

            // 4. Search the points using laser data
            for( it= laser->begin(); it!= laser->end(); it++){

              // Note: Changing the axis to be compatible with kinect axis data
              searchPoint.x = it->x; // z to x
              searchPoint.y = it->y;
              searchPoint.z = it->z; // x to z
              // K nearest neighbor search

              int K = 10;

              std::vector<int> pointIdxNKNSearch(K);
              std::vector<float> pointNKNSquaredDistance(K);

              if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
              {
                // Validation of each nearest point found
                for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                  
                  // Green point (Good ones) - Validation
                  // if(cloud->points[ pointIdxNKNSearch[i]].g==255 && searchPoint.y <= std::stof (getenv("HEIGHT_THRESHOLD"))
                  if(cloud->points[ pointIdxNKNSearch[i]].g==255)
                  {
                    
                    // Sum the distance from laser search point to all nearest points
                    tempDistanceCount += distance(cloud->points[ pointIdxNKNSearch[i]].x,
                                                  cloud->points[ pointIdxNKNSearch[i]].y,
                                                  cloud->points[ pointIdxNKNSearch[i]].z,
                                                  searchPoint.x,
                                                  searchPoint.y,
                                                  searchPoint.z);
                    // Count number of green points
                    tempGreenCount++;

                  }

                  // Red point (Not good ones) - Validation
                  if(cloud->points[ pointIdxNKNSearch[i]].r==255 && searchPoint.y <= std::stof (getenv("HEIGHT_THRESHOLD"))
                  ){
                    // Count number of red points
                    tempRedCount++;
                  }  

                }
              }

              // Update controls to filter just the nearest intersection point between laser ray and terrain
              if(tempGreenCount>=greenCount && tempDistanceCount <= distanceCount && tempDistanceCount > 0){
                    targetX = searchPoint.x;
                    targetY = searchPoint.y;
                    targetZ = searchPoint.z;
                    greenCount=tempGreenCount;
                    distanceCount=tempDistanceCount;
              }
              tempRedCount = 0;
              tempGreenCount = 0;
              tempDistanceCount = 0;
            }

            // Publish the intersection point
            PointCloud::Ptr intersection_cloud (new PointCloud);
            pcl::PointXYZRGBA point_intersection;
            point_intersection.x = targetX;
            point_intersection.y = targetY;
            point_intersection.z = targetZ;
            point_intersection.r = 255;
            point_intersection.g = 153;
            point_intersection.b = 0;

            intersection_cloud->push_back(point_intersection);
                        
            sensor_msgs::PointCloud2 msgcloud;
            pcl::toROSMsg(*intersection_cloud, msgcloud); 
            std::string tf_frame;
            tf_frame = "base_link";
            msgcloud.header.frame_id = tf_frame;
            msgcloud.header.stamp = ros::Time::now();
            pub_sim_laser_intersection_.publish (msgcloud);

            // 5. Determine the passage condition
            std_msgs::String msg;

            if(greenCount >= 10){
            // if((greenCount / (greenCount+redCount)) >= std::stof (getenv("MINIMUM_GREEN_POINTS"))){
              if(classifier_ready == true){
                generate_path_line(targetX, targetY, targetZ);
              }
              msg.data = "safe";
              cout.precision(dbl::max_digits10);
              cout << "safe total - red: " << redCount << endl;
              cout << "safe total - green: " << greenCount << endl;
              

            }else{
              msg.data = "unsafe";
              cout.precision(dbl::max_digits10);
              cout << "safe total - red: " << redCount << endl;
              cout << "safe total - green: " << greenCount << endl;
              
            }

            // 6. Publish the passage condition
           pub_passage_condition_.publish (msg);

           printf("classify_passage_condition \n");
           printf("***** Finished ******* \n");
        }


        /*
          Generate virtual ray laser data in Point Cloud PointCloud::Ptr
        */
        
        PointCloud::Ptr generate_virtual_ray_laser_data_in_point_cloud(double pan, double tilt, double origin_x, double origin_y, double origin_z){
            
            // 1. Calculate target point
            double target_x;
            double target_y;
            double target_z;
            double origin_z_calc = origin_z + std::stod (getenv("LASER_Z_GAP"));

            // if(tilt<0.4){
            //   target_x = origin_x + 5.0;
            //   target_y = origin_z * sin(pan) / tan(tilt);
            //   target_z = origin_z * -1;              
            // }else{
            if(tilt==0){tilt = 1;};

            target_x = origin_z_calc * cos(pan) / tan(tilt);
            target_y = origin_z_calc * sin(pan) / tan(tilt);
            target_z = origin_z_calc * -1;
            // }

            cout << "***** pan & tilt ******" << endl;
            cout.precision(dbl::max_digits10);
            cout << "Pan: " << pan << " - Tilt: " << tilt << endl;

            cout << "***** origin ******" << endl;
            cout.precision(dbl::max_digits10);
            cout << "X: " << origin_x << " - Y: " << origin_y << " - Z: " << origin_z << endl;            
 
            cout << "***** target ******" << endl;
            cout.precision(dbl::max_digits10);
            cout << "X: " << target_x << " - Y: " << target_y << " - Z: " << target_z << endl; 

            int rgb [3] = { 255, 85, 0 }; 

            // 2. Generate the point cloud ray
            if((target_x - origin_x)>8.0){
              target_x = 5.0;
              rgb[0] = 94;
              rgb[1] = 94;
              rgb[2] = 94;
            }

            PointCloud::Ptr cloud = generate_line_points(origin_x, origin_y, origin_z, target_x, target_y, target_z);
            
            PointCloud::Ptr target_cloud (new PointCloud);
            pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;
            pcl::PointXYZRGBA point_new;
            
            for( it= cloud->begin(); it!= cloud->end(); it++){
              // Add to the robot path point cloud
              point_new.x = it->x;
              point_new.y = it->y;
              point_new.z = it->z;                       
              point_new.r = rgb[0];
              point_new.g = rgb[1];
              point_new.b = rgb[2];
              point_new.a = 255;                      
              target_cloud->push_back(point_new);
            }     
            PassageClassificationProcess::laser_data_ready = true;
            return target_cloud;

        }

        /*
        * This function will process the callback for laser orientation
        */
        void laser_data_orientation_callback(const sensor_msgs::JointStateConstPtr& msg){

          PassageClassificationProcess::laser_tilt = msg->position[0];
          PassageClassificationProcess::laser_pan = msg->position[1];

        }

        /*
        * This function will process the callback for laser sim orientation pan
        */
        void laser_sim_pan_data_orientation_callback(const std_msgs::Float64ConstPtr& msg){

          PassageClassificationProcess::laser_sim_pan = msg->data;

        }        

        /*
        * This function will process the callback for laser sim orientation tilt
        */
        void laser_sim_tilt_data_orientation_callback(const std_msgs::Float64ConstPtr& msg){

          PassageClassificationProcess::laser_sim_tilt = msg->data;

        } 

        /*
        * This function will calculate the distance from robot to the target point
        * and publish the visualization of this path
        */
        void generate_path_line(double targetX, double targetY, double targetZ){

            classifier_ready == false;

            // 2. Load the UPD cloud to kdtree object 
            kdtree.setInputCloud (PassageClassificationProcess::upd_data);

            // 3. Iterate the laser points and search them using kdtree
            pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;

            // Robot path point cloud
            PointCloud::Ptr target_path_cloud (new PointCloud);
            pcl::PointXYZRGBA point2;
            pcl::PointXYZRGBA point_remain;
            pcl::PointXYZRGBA point_candidate;

            point2.x = targetX;
            point2.y = targetY;
            point2.z = targetZ;                     
            point2.r = 254;
            point2.g = 0;
            point2.b = 0;
            point2.a = 255;                      
            target_path_cloud->push_back(point2);

            float distance_previous = 0;
            float distance_candidate = INFINITY;
            float distance_total = 0;
            float distance_temp = 0;

            bool process = true;
            searchPoint.x = targetX;
            searchPoint.y = targetY;
            searchPoint.z = targetZ;

            // 4. Search the points
            while(process){

              int K = std::stoi (getenv("GENERATE_PATH_LINE_K"));

              std::vector<int> pointIdxNKNSearch(K);
              std::vector<float> pointNKNSquaredDistance(K);

              if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
              {
                
                for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                  
                  if(PassageClassificationProcess::upd_data->points[ pointIdxNKNSearch[i]].g==255){
                   
                    // Calculate temp distance
                    distance_temp = distance(
                        0, 0, -0.35,
                        PassageClassificationProcess::upd_data->points[ pointIdxNKNSearch[i]].x, 
                        PassageClassificationProcess::upd_data->points[ pointIdxNKNSearch[i]].y, 
                        PassageClassificationProcess::upd_data->points[ pointIdxNKNSearch[i]].z
                    );

                    // Revalidate the candidate point
                    if(distance_temp < distance_candidate){
                      distance_candidate = distance_temp;
                      point_candidate = PassageClassificationProcess::upd_data->points[ pointIdxNKNSearch[i]];
                    }

                  }

                }

                // Condition: The nearest point was found already, stop the processing
                if( ( distance_candidate >= distance_previous || distance_candidate <= std::stod (getenv("POINT_CLOUD_TO_ROBOT_CENTER") )) && distance_previous > 0){
                  process = false;
                  cout << "***** Distance Calculation ******" << endl;
                  cout.precision(dbl::max_digits10);
                  cout << "Distance Total: " << distance_total << endl;
                }else{
                  distance_total += fabs((fabs(distance_candidate) - fabs(distance_previous)));
                  cout << "***** Distance Calculation Partial ******" << endl;
                  cout.precision(dbl::max_digits10);
                  // cout << "Distance Previous: " << distance_previous << endl;
                  // cout << "Distance Candidate: " << distance_candidate << endl;
                  // cout << "Difference: " << fabs((fabs(distance_candidate) - fabs(distance_previous))) << endl;
                  cout << "Distance Total: " << distance_total << endl;
                  distance_previous = distance_candidate;

                  // Add to the robot path point cloud
                  point2.x = point_candidate.x;
                  point2.y = point_candidate.y;
                  point2.z = point_candidate.z;                   
                  point2.r = 0;
                  point2.g = 0;
                  point2.b = 254;
                  point2.a = 255;                      
                  target_path_cloud->push_back(point2);

                  searchPoint.x = point_candidate.x;
                  searchPoint.y = point_candidate.y;
                  searchPoint.z = point_candidate.z;
                }

              }
              if(process == false){

                //distance_total += distance_candidate;
                cout << "Distance Total: " << distance_total << endl;

                PointCloud::Ptr remain (new PointCloud);
                remain = generate_line_points(0,0,-0.35, point2.x, point2.y, point2.z);
             
                for( it= remain->begin(); it!= remain->end(); it++){
                  // Add to the robot path point cloud
                  point_remain.x = it->x;
                  point_remain.y = it->y;
                  point_remain.z = it->z;                      
                  point_remain.r = 0;
                  point_remain.g = 0;
                  point_remain.b = 254;
                  point_remain.a = 255;                      
                  target_path_cloud->push_back(point_remain);
                }             

                //process = false;
                

              }else{
                distance_candidate = INFINITY;

              }

            }

            // 6. Publish the target path
            sensor_msgs::PointCloud2 msgcloud;
            pcl::toROSMsg(*target_path_cloud, msgcloud); 
            std::string tf_frame;
            tf_frame = "base_link";
            // nh_.param("frame_id", tf_frame, std::string("/base_link"));
            msgcloud.header.frame_id = tf_frame;
            msgcloud.header.stamp = ros::Time::now();
            target_path_.publish (msgcloud);

            // 7. Publish the target distance
            cout.precision(dbl::max_digits10);
            cout << "Distance Total: " << distance_total << endl;
            std_msgs::Float64 msg_target_distance;
            msg_target_distance.data = distance_total;
            pub_target_distance_.publish(msg_target_distance);
            classifier_ready = true;

        }


        /*
        * This is a support function to calculae the distance between 2 points
        */
        float distance(float x1, float y1, float z1, float x2, float y2, float z2)
        {
            float d = sqrt(pow(x2 - x1, 2) +
                        pow(y2 - y1, 2) +
                        pow(z2 - z1, 2) * 1.0);
            std::cout << std::fixed;
            std::cout << std::setprecision(2);
            // cout << " Distance is: " << d;
            return d;
        }

        /*
        * This is a support function to generate a point cloud line points between two points
        */
        PointCloud::Ptr generate_line_points(float x1, float y1, float z1, float x2, float y2, float z2){

          pcl::PointXYZRGBA point2;          
          pcl::PointXYZRGB laser_vector_points;

          float A [3] = {x1,y1,z1};
          float B [3] = {x2,y2,z2};
          float C [3] = {fabsf(B[0]-A[0]), fabsf(B[1]-A[1]), fabsf(B[2]-A[2])};

          int x_direction = 1;
          int y_direction = 1;
          int z_direction = 1;

          if((B[0]<A[0])){x_direction=-1;};
          if((B[1]<A[1])){y_direction=-1;};
          if((B[2]<A[2])){z_direction=-1;};
          
          float max_difference = C[0];
          if(C[1] > max_difference){ max_difference = C[1];};
          if(C[2] > max_difference){ max_difference = C[2];};    
          max_difference = max_difference / 0.05;

          float line_vector_x [(int) max_difference +1] = {};
          float line_vector_y [(int) max_difference +1] = {};
          float line_vector_z [(int) max_difference +1] = {};

          float step_x = C[0] / max_difference;
          float step_y = C[1] / max_difference;
          float step_z = C[2] / max_difference;

          line_vector_x[0] = A[0];
          line_vector_y[0] = A[1];
          line_vector_z[0] = A[2];


          for(int i=1; i<=max_difference;i++){

            if(step_x==0){ line_vector_x[i] = A[0]; }else{line_vector_x[i] = line_vector_x[i-1] + (step_x*x_direction);};
            if(step_y==0){ line_vector_y[i] = A[1]; }else{line_vector_y[i] = line_vector_y[i-1] + (step_y*y_direction);};
            if(step_z==0){ line_vector_z[i] = A[2]; }else{line_vector_z[i] = line_vector_z[i-1] + (step_z*z_direction);};

          }
          // Point Cloud generation
          PointCloud::Ptr laser_line_cloud (new PointCloud);
          pcl::PointXYZRGBA point;

          for( int i = 0; i< (int) max_difference; i++){
              point.x = line_vector_x[i];
              point.y = line_vector_y[i];
              point.z = line_vector_z[i];
              point.r = 0;
              point.g = 0;
              point.b = 255;
              point.a = 1;
              laser_line_cloud->push_back(point);
          }

          return laser_line_cloud;

        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_upd_;
        ros::Subscriber sub_laser_data_;
        ros::Subscriber sub_laser_orientation_;
        ros::Subscriber sub_laser_sim_pan_data_;
        ros::Subscriber sub_laser_sim_tilt_data_;
        ros::Publisher pub_passage_condition_;
        ros::Publisher target_path_;
        ros::Publisher laser_simulated_ray_;
        ros::Publisher pub_target_distance_;
        ros::Publisher pub_sim_laser_intersection_;
        PointCloud::Ptr upd_data;
        PointCloud::Ptr laser_data;
        pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
        pcl::PointXYZRGBA searchPoint;        
        double laser_pan = 0.0;
        double laser_tilt = 0.0; 
        double laser_sim_pan = 0.0;
        double laser_sim_tilt = 0.0;          
  
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