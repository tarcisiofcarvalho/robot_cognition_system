#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
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

// PCL Cloud type definition
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud2;
typedef pcl::PointCloud<pcl::PointSurfel> PointCloudSurfel;
typedef pcl::PointXYZRGBA PointT;



pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
// pcl::visualization::CloudViewer viewer ("Show UPD");
pcl::visualization::CloudViewer viewer2 ("Show Points");
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
// pcl::visualization::CloudViewer viewerUPD ("Show UPD");

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

/* This function receives the OpenNI data from a kinect device, 
    filter and classify the point cloud data with UPD classifier, 
    and publish the UPD classified data */
void process_upd(const PointCloud::ConstPtr& msg)
{
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // BOOST_FOREACH (const pcl::PointXYZRGBA& pt, msg->points)
    // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    // 1. Converting from const to non const boost
    PointCloud::Ptr msg2 = boost::const_pointer_cast<PointCloud>(msg);

    // if (!viewer2.wasStopped()){
    //       viewer2.showCloud (msg2);
    // }
    // pcl::PointXYZRGBA point;
    // int counter = 0;
    // pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;
    // PointCloud::Ptr newMsg (new PointCloud);
    // for( it= msg2->begin(); it!= msg2->end(); it++){
    // if(counter%50==0)
    //     point.x = it->x;
    //     point.y = it->y;
    //     point.z = it->z;
    //     point.r = it->r;
    //     point.g = it->g;
    //     point.b = it->b;
    //     point.a = it->a;
    //     newMsg->push_back(point);
    //     // outputCloud->push_back (pcl::PointXYZ (it->x, it->y, it->z));
    //     counter ++;   
    // }
    // 2. New UPD object
    upd *m_upd;
    m_upd = new upd;

    // 3. Apply filters
    bool showStatistics = false;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
    PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> ();    
    // ****** Remove NAN data *******
    PointCloud::Ptr outputCloud (new PointCloud);
    PointCloud::Ptr cloudOut (new PointCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*msg2,*outputCloud, indices);

    // ******* Compress point cloud ********
    // stringstream to store compressed point cloud
    std::stringstream compressedData;

    // compress point cloud
    PointCloudEncoder->encodePointCloud (outputCloud, compressedData);

    // decompress point cloud
    PointCloudDecoder->decodePointCloud (compressedData, cloudOut);

    // Voxel Grid - points reduction
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloudOut);
    sor.setLeafSize (std::stof(getenv("UPD_VOX_GRID_LEAF_X")), std::stof(getenv("UPD_VOX_GRID_LEAF_Y")), std::stof(getenv("UPD_VOX_GRID_LEAF_Z")));
    PointCloud::Ptr cloud_filtered (new PointCloud);
    sor.filter (*cloud_filtered);

    // ****** Save point cloud data file *******
    // pcl::io::savePCDFile("test_pcd_rgba_compressed_may_25th.pcd", *cloud_filtered);

    // 3. Set input cloud
    // PointCloudSurfel::Ptr updInput (new PointCloudSurfel);

    // pcl::copyPointCloud(*cloud_filtered, *updInput);

    m_upd->setInputCloud(cloud_filtered);

    printf("Info: setInpuCloud");
    // 3. Set radius
    m_upd->setSearchRadius(std::stod(getenv("UPD_SEARCH_RADIUS")));

    // 4. Run UPD radius
    m_upd->runUPD_radius();

    // 5. Get UPD
    m_upd->getUPD();

    // 6. Get the colored map
    
    // 6.1 Prepare the parameters data
    PointCloud::Ptr m_cloud_color_UPD (new PointCloud);

    double unevenness = std::stod (getenv("UPD_UNEVENNESS"));
    double unevennessMax = std::stod (getenv("UPD_UNEVENNESS_MAX"));
    double radAngle = (std::stod(getenv("UPD_RAD_ANGL")) * M_PI / 180);
    
    // 6.2 Get colored map
    m_upd->setColorMapType(false);
    m_upd->getAsColorMap(m_cloud_color_UPD,
                           (unevenness)/unevennessMax,
						   radAngle);
    //  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    // if (!viewer.wasStopped ()) {
    //     viewer.removePointCloud();
    //     pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_color(m_cloud_color_UPD);
    //     viewer.addPointCloud (m_cloud_color_UPD, rgb_color, "cloud");
    //     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    //     viewer.spin ();
    // }

    if (!viewer2.wasStopped()){
          viewer2.showCloud (m_cloud_color_UPD);
    }
    
    // 7. Publish the UPD classified point clouds
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);
    printf("Info: 4. UPD published pointCloud \n");
    pub.publish (m_cloud_color_UPD);

    sensor_msgs::PointCloud2 msgcloud;
    pcl::toROSMsg(*m_cloud_color_UPD.get(), msgcloud); 

    ros::NodeHandle nh2;
    std::string tf_frame;
    tf_frame = "/base_link";
    nh2.param("frame_id", tf_frame, std::string("/base_link"));
    msgcloud.header.frame_id = tf_frame;
    msgcloud.header.stamp = ros::Time::now();
    ros::Publisher pub2 = nh2.advertise<sensor_msgs::PointCloud2> ("upd_point_cloud_classification_rviz", 1);
    printf("Info: 4. UPD published msgCloud \n");
    pub2.publish (msgcloud);
    
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spinOnce();

}

int main(int argc, char** argv){
    
    // 0. Printing the parameters
    printf("===================================================");
    printf("Info: Setup parameters \n");
    printf("POINT_CLOUD_API = %s \n", getenv("POINT_CLOUD_API"));
    printf("UPD_UNEVENNESS = %s \n", getenv("UPD_UNEVENNESS"));
    printf("UPD_UNEVENNESS_MAX = %s \n", getenv("UPD_UNEVENNESS_MAX"));
    printf("UPD_SEARCH_RADIUS = %s \n", getenv("UPD_SEARCH_RADIUS"));
    printf("UPD_VOX_GRID_LEAF_X = %s \n", getenv("UPD_VOX_GRID_LEAF_X"));
    printf("UPD_VOX_GRID_LEAF_Y = %s \n", getenv("UPD_VOX_GRID_LEAF_Y"));
    printf("UPD_VOX_GRID_LEAF_Z = %s \n", getenv("UPD_VOX_GRID_LEAF_Z"));
    printf("UPD_REDUCTION_PERCENT = %s \n", getenv("UPD_REDUCTION_PERCENT"));
    printf("=================================================== \n");

    // 1. ROS Init
    printf("Info: 1. ROS Init \n");
    ros::init (argc, argv, "upd_node");

    // 2. Subscriber OpenNI
    printf("Info: 2. Subscriber OpenNI \n");
    ros::NodeHandle nhk;

    // 3. Publishing UPD data 
    printf("Info: 3. Publishing UPD data \n");
    ros::Subscriber sub = nhk.subscribe<PointCloud>("/camera/depth/points", 1, process_upd);

    ros::spin();
}

// class UPD_Publisher{
//     public:
//         static const rosNode nh;
//         static const rosPublisher pub;
//         static const rosNode nhk;
//         static const rosSubscriber sub;
//         // UPD_Publisher(){
//         //     // 1. Setup UPD publisher that will provide classified points
//         //     pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);

//         //     // 2. Setup subscriber for Kinect Point Cloud data and run the callback process
//         //     sub = nhk.subscribe<PointCloud>("/camera/depth/points", 1, process);

//         //     // 3. ROS run
//         //     ros::Rate loop_rate(4);
//         //     ros::spin();

//         // }
//         static void process(const PointCloud::ConstPtr& msg){
//             // cout << "publisher callback processing.....";
//             printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//             BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//             printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

//             // 1. Publish the UPD data
//             //pub.publish (msg);
//         };

//         static void start(){

//             // 1. Setup UPD publisher that will provide classified points
//             pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);

//             // 2. Setup subscriber for Kinect Point Cloud data and run the callback process
//             sub = nhk.subscribe<PointCloud>("/camera/depth/points", 1, process);

//             // 3. ROS run
//             ros::Rate loop_rate(4);
//             ros::spin();

//         }

// };

// int main(int argc, char** argv){
    
//     ros::init (argc, argv, "upd_node");

//     UPD_Publisher upd_publisher;

//     upd_publisher.start();    

//     // upd_publisher();

// }