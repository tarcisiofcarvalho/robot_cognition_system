#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <upd.h>
#include <export.h>
#include <math.h>
#include <stdlib.h>
#include <string> 
#include <fstream>
using namespace std;

#define _USE_MATH_DEFINES
#define PI 3.14159265

// PCL Cloud type definition
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud2;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointSurfel> PointCloudSurfel;
typedef pcl::PointXYZRGBA PointT;

pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;


/*
Return a RGB colour value given a scalar v in the range [vmin,vmax]
In this case each colour component ranges from 0 (no contribution) to
1 (fully saturated), modifications for other ranges is trivial.
The colour is clipped at the end of the scales if v is outside
the range [vmin,vmax]
*/
uint32_t GiveJetColour(double _value, double _vmin, double _vmax)
{
	//COLOUR c = { 1.0, 1.0, 1.0 }; // white
	double R = 1.0;// byte
	double G = 1.0;// byte
	double B = 1.0;// byte
	double dv;
	//cout << "upd::GiveJetColourthe::MESSAGE:  RGB value is : " << _value << endl;

	if (_value < _vmin)
		_value = _vmin;
	if (_value > _vmax)
		_value = _vmax;
	dv = _vmax - _vmin;

	if (_value < (_vmin + 0.25 * dv)) {
		R = 0;
		G = 4 * (_value - _vmin) / dv;
	}
	else if (_value < (_vmin + 0.5 * dv)) {
		R = 0;
		B = 1 + 4 * (_vmin + 0.25 * dv - _value) / dv;
	}
	else if (_value < (_vmin + 0.75 * dv)) {
		R = 4 * (_value - _vmin - 0.5 * dv) / dv;
		B = 0;
	}
	else {
		G = 1 + 4 * (_vmin + 0.75 * dv - _value) / dv;
		B = 0;
	}

	//cout << "upd::GiveJetColourthe::MESSAGE:  RGB value is : " << R << "  " << G << "  " << B << endl;

	uint8_t _R = int(ceil(255 * R));
	uint8_t _G = int(ceil(255 * G));
	uint8_t _B = int(ceil(255 * B));

	//cout << "upd::GiveJetColourthe::MESSAGE:  RGB value is : " << _R << "  " << _G << "  " << _B << endl;

	return (_R << 16) | (_G << 8) | _B;
}

/* this function gives 1D linear RGB color gradient
 color is proportional to position
 position  <0;1>
position means position of color in color gradient */ 
uint32_t GiveRainbowColor(float position)
{
  if (position>1)position=1;//position-int(position);
  // if position > 1 then we have repetition of colors
  // it maybe useful
  uint8_t R = 0;// byte
  uint8_t G = 0;// byte
  uint8_t B = 0;// byte
  int nmax=6;// number of color bars
  float m=nmax* position;
  int n=int(m); // integer of m
  float f=m-n;  // fraction of m
  uint8_t t=int(f*255);


    switch( n){
    case 0:
        {
        R = 0;
        G = 255;
        B = t;
        break;
        }

    case 1:
        {
        R = 0;
        G = 255 - t;
        B = 255;
        break;
        }
    case 2:
        {
        R = t;
        G = 0;
        B = 255;
        break;
        }
    case 3:
        {
        R = 255;
        G = 0;
        B = 255 - t;
        break;
        }
    case 4:
        {
        R = 255;
        G = t;
        B = 0;
        break;
        }
    case 5: {
        R = 255 - t;
        G = 255;
        B = 0;
        break;
        }
    case 6:
        {
        R = 0;
        G = 255;
        B = 0;
        break;
        }

    } // case


    return (R << 16) | (G << 8) | B;
}

// UPD processing function
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getAsColorMap(double _traversability_index_threshold, 
                   double _suitable_angle,
                   pcl::PointCloud<pcl::PointSurfel>::Ptr& UPD_cloud){
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _rgb_upd_cloud;

    bool m_colorMap;

	_rgb_upd_cloud->resize(UPD_cloud->size());
	for (unsigned int i = 0; i< UPD_cloud->size(); i++)
	{
     _rgb_upd_cloud->points[i].x = UPD_cloud->points[i].x;
     _rgb_upd_cloud->points[i].y = UPD_cloud->points[i].y;
     _rgb_upd_cloud->points[i].z = UPD_cloud->points[i].z;
	 float rgb_value = 0;

	 if (tan(UPD_cloud->points[i].normal_x/UPD_cloud->points[i].normal_y) > tan(_suitable_angle) ||
		 tan(UPD_cloud->points[i].normal_z/UPD_cloud->points[i].normal_y) > tan(_suitable_angle))
	 {
	 _rgb_upd_cloud->points[i].rgba = 0x00FF0000;   //set color to RED in case of orientation fail
	 }
	 else {
		 if(UPD_cloud->points[i].radius > _traversability_index_threshold )//&& UPD_cloud->points[i].curvature<0.9999)
		{
			rgb_value = (UPD_cloud->points[i].radius-_traversability_index_threshold)/(1-_traversability_index_threshold);
			m_colorMap = false;
			if (m_colorMap == true)
			{
				_rgb_upd_cloud->points[i].rgba = GiveJetColour(1 - rgb_value, 0.0, 1.0);
			}
			else
			{
				_rgb_upd_cloud->points[i].rgba = GiveRainbowColor(rgb_value);
			}
		 }

		else
		{
			rgb_value = (UPD_cloud->points[i].radius-0.7)/0.3;
			_rgb_upd_cloud->points[i].rgba = 0x00000000;//GiveRainbowColor(rgb_value);
		}

	 }

	}
    return _rgb_upd_cloud;

}

class UPDProcess{
    public:
        UPDProcess(){
            // 1. Subscribe to the Kinect Node
            sub_kinect_ = nh_.subscribe<PointCloud>("/camera/depth/points", 1, &UPDProcess::callback, this);

            // 2. Publish UPD data
            pub_upd_ = nh_.advertise<PointCloud> ("upd_point_cloud_classification", 1);

            // 3. Publish UPD Rviz data
            pub_upd_rviz_ = nh_.advertise<sensor_msgs::PointCloud2> ("upd_point_cloud_classification_rviz", 1);
            pub_upd_rviz_base_1_ = nh_.advertise<sensor_msgs::PointCloud2> ("upd_point_cloud_classification_rviz_base_1", 1);
            pub_upd_rviz_base_2_ = nh_.advertise<sensor_msgs::PointCloud2> ("upd_point_cloud_classification_rviz_base_2", 1);
        }
    
        void callback(const PointCloud::ConstPtr& msg){
            // 1. Converting from const to non const boost
            PointCloud::Ptr msg2 = boost::const_pointer_cast<PointCloud>(msg);

            // 2. Defining UPD
            upd *m_upd;
            m_upd = new upd;

            // 3. Appling filters
            bool showStatistics = false;

            // 3.1 Compression definition
            pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

            // 3.2 Encoder and Decoder definition
            PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
            PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> ();    

            // 3.3 Removing NAN data
            PointCloud::Ptr outputCloud (new PointCloud);
            PointCloud::Ptr cloudOut (new PointCloud);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*msg2,*outputCloud, indices);

            // 3.4 Defining compress data
            std::stringstream compressedData;

            // 3.5 Compress the point cloud
            PointCloudEncoder->encodePointCloud (outputCloud, compressedData);

            // 3.5 Decompress the point cloud
            PointCloudDecoder->decodePointCloud (compressedData, cloudOut);

            delete PointCloudEncoder;
            delete PointCloudDecoder;

            // 3.6 Vox grid reduction
            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
            sor.setInputCloud (cloudOut);
            sor.setLeafSize (std::stof(getenv("UPD_VOX_GRID_LEAF_X")), std::stof(getenv("UPD_VOX_GRID_LEAF_Y")), std::stof(getenv("UPD_VOX_GRID_LEAF_Z")));
            PointCloud::Ptr cloud_filtered (new PointCloud);
            sor.filter (*cloud_filtered);

            // 4. Set filtered point cloud in UPD 
            m_upd->setInputCloud(cloud_filtered);

            // 5. Set UPD radius
            m_upd->setSearchRadius(std::stod(getenv("UPD_SEARCH_RADIUS")));

            // 6. Run UPD
            m_upd->runUPD_radius();

            // 7. Get UPD
            m_upd->getUPD();

            // 8. Get the colored map
            
            // 8.1 Prepare the parameters data
            PointCloud::Ptr m_cloud_color_UPD (new PointCloud);

            double unevenness = std::stod (getenv("UPD_UNEVENNESS"));
            double unevennessMax = std::stod (getenv("UPD_UNEVENNESS_MAX"));
            double radAngle = (std::stod(getenv("UPD_DEG_ANGL")) * M_PI / 180);
            
            // 8.2 Get UPD colored map
            m_upd->setColorMapType(false);
            m_upd->getAsColorMap(m_cloud_color_UPD,
                                (unevenness)/unevennessMax,
                                radAngle);
            
            // 9. Publish the UPD classified point clouds to Rviz
            PointCloudRGB::Ptr upd_temp (new PointCloudRGB);
            pcl::PointXYZRGB point_temp;
            pcl::PointCloud<pcl::PointXYZRGBA>::iterator it2;

            for( it2= m_cloud_color_UPD->begin(); it2!= m_cloud_color_UPD->end(); it2++){
                point_temp.x = it2->x;
                point_temp.y = it2->y;
                point_temp.z = it2->z;
                point_temp.r = it2->r;
                point_temp.g = it2->g;
                point_temp.b = it2->b;
                upd_temp->push_back(point_temp);
            }

            sensor_msgs::PointCloud2 msgcloud;
            pcl::toROSMsg(*upd_temp, msgcloud); 
            std::string tf_frame;
            tf_frame = "camera_depth_optical_frame";
            // nh_.param("frame_id", tf_frame, std::string("/base_link"));
            msgcloud.header.frame_id = tf_frame;
            msgcloud.header.stamp = ros::Time::now();
            pub_upd_rviz_.publish (msgcloud);


            
            // 11. Publish the UPD classified point clouds
            upd_temp = transform_kinect_to_base(upd_temp);
            pub_upd_.publish (upd_temp);
            
            sensor_msgs::PointCloud2 msgcloud2;
            pcl::toROSMsg(*upd_temp, msgcloud2); 
            tf_frame = "base_link";
            // nh_.param("frame_id", tf_frame, std::string("/base_link"));
            msgcloud2.header.frame_id = tf_frame;
            msgcloud2.header.stamp = ros::Time::now();
            pub_upd_rviz_base_2_.publish (msgcloud2);

            delete m_upd;

            printf("UPD Processed \n");
            ros::Rate loop_rate(std::stod (getenv("ROS_UPD_LOOP_RATE")));
            loop_rate.sleep();

        }

        PointCloudRGB::Ptr transform_kinect_to_base(PointCloudRGB::Ptr cloud){


            // 9. Publish the UPD classified point clouds to Rviz
            PointCloudRGB::Ptr cloud_temp (new PointCloudRGB);
            pcl::PointXYZRGB point_temp;
            pcl::PointCloud<pcl::PointXYZRGB>::iterator it2;
            for( it2= cloud->begin(); it2!= cloud->end(); it2++){
                point_temp.x = it2->z;
                point_temp.y = -it2->x;
                point_temp.z = -it2->y;
                point_temp.r = it2->r;
                point_temp.g = it2->g;
                point_temp.b = it2->b;
                cloud_temp->push_back(point_temp);
            }

            // Translate to base frame
            Eigen::Matrix<float, 4, 4> translationMatrix;

            translationMatrix(0,0) = 1;
            translationMatrix(0,1) = 0;
            translationMatrix(0,2) = 0;
            translationMatrix(0,3) = std::stof (getenv("KINECT_TO_BASE_X"));

            translationMatrix(1,0) = 0;
            translationMatrix(1,1) = 1;
            translationMatrix(1,2) = 0;
            translationMatrix(1,3) = std::stof (getenv("KINECT_TO_BASE_Y"));;

            translationMatrix(2,0) = 0;
            translationMatrix(2,1) = 0;
            translationMatrix(2,2) = 1;
            translationMatrix(2,3) = std::stof (getenv("KINECT_TO_BASE_Z"));;    

            translationMatrix(3,0) = 0;
            translationMatrix(3,1) = 0;
            translationMatrix(3,2) = 0;
            translationMatrix(3,3) = 1;
          
            pcl::transformPointCloud(*cloud_temp, *cloud_temp, translationMatrix);

            // // Rotate to base frame
            // Eigen::Matrix4f eRot;
            // Eigen::Quaternionf PCRot;
            // Eigen::Vector3f PCTrans;
            // Eigen::Matrix<float, 4, 4> rotMatrix;

            // // Counterclockwise rotation around y axis
            // rotMatrix(0,0) = cos(PI);
            // rotMatrix(0,1) = 0;
            // rotMatrix(0,2) = sin(PI);
            // rotMatrix(0,3) = 0;

            // rotMatrix(1,0) = 0;
            // rotMatrix(1,1) = 1;
            // rotMatrix(1,2) = 0;
            // rotMatrix(1,3) = 0;

            // rotMatrix(2,0) = -sin(PI);
            // rotMatrix(2,1) = 0;
            // rotMatrix(2,2) = cos(PI);
            // rotMatrix(2,3) = 0;    

            // rotMatrix(3,0) = 0;
            // rotMatrix(3,1) = 0;
            // rotMatrix(3,2) = 0;
            // rotMatrix(3,3) = 1;

            
            // pcl::transformPointCloud(*cloud, *cloud, rotMatrix);


            // // Counterclockwise rotation around z axis
            // rotMatrix(0,0) = cos(-PI);
            // rotMatrix(0,1) = -sin(-PI);
            // rotMatrix(0,2) = 0;
            // rotMatrix(0,3) = 0;

            // rotMatrix(1,0) = sin(-PI);
            // rotMatrix(1,1) = cos(-PI);
            // rotMatrix(1,2) = 0;
            // rotMatrix(1,3) = 0;

            // rotMatrix(2,0) = 0;
            // rotMatrix(2,1) = 0;
            // rotMatrix(2,2) = 1;
            // rotMatrix(2,3) = 0;    

            // rotMatrix(3,0) = 0;
            // rotMatrix(3,1) = 0;
            // rotMatrix(3,2) = 0;
            // rotMatrix(3,3) = 1;

            
            // pcl::transformPointCloud(*cloud, *cloud, rotMatrix);

            return cloud_temp;

        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_kinect_;
        ros::Publisher pub_upd_;
        ros::Publisher pub_upd_rviz_;
        ros::Publisher pub_upd_rviz_base_1_;
        ros::Publisher pub_upd_rviz_base_2_;
};



int main(int argc, char** argv){
    
    // 0. Printing the parameters
    printf("=================================================== \n");
    printf("Info: Setup parameters \n");
    printf("POINT_CLOUD_API = %s \n", getenv("POINT_CLOUD_API"));
    printf("UPD_UNEVENNESS = %s \n", getenv("UPD_UNEVENNESS"));
    printf("UPD_UNEVENNESS_MAX = %s \n", getenv("UPD_UNEVENNESS_MAX"));
    printf("UPD_SEARCH_RADIUS = %s \n", getenv("UPD_SEARCH_RADIUS"));
    printf("UPD_DEG_ANGL = %s \n", getenv("UPD_DEG_ANGL"));
    printf("UPD_VOX_GRID_LEAF_X = %s \n", getenv("UPD_VOX_GRID_LEAF_X"));
    printf("UPD_VOX_GRID_LEAF_Y = %s \n", getenv("UPD_VOX_GRID_LEAF_Y"));
    printf("UPD_VOX_GRID_LEAF_Z = %s \n", getenv("UPD_VOX_GRID_LEAF_Z"));
    printf("UPD_REDUCTION_PERCENT = %s \n", getenv("UPD_REDUCTION_PERCENT"));
    printf("ROS_UPD_LOOP_RATE = %s \n", getenv("ROS_UPD_LOOP_RATE"));
    printf("=================================================== \n");

    // 1. ROS Init
    printf("Info: 1. ROS Init \n");
    ros::init (argc, argv, "upd_node");

    // 2. Publishing UPD data 
    printf("Info: 2. Publishing UPD data \n");
    UPDProcess updProcess;

    // 3. Spin ROS cycle
    ros::spin();
}