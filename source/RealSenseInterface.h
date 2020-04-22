#pragma once
#include <vector>
#include <string>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include<opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>

//#include<pcl/registration/transforms.h>
//
//#include <pcl/filters/approximate_voxel_grid.h>


#include"TimeString.h"

using namespace std;

class CRealSenseInterface
{
	rs2::colorizer M_colorizer;

	rs2::pipeline M_pipe;
	rs2::config M_cfg;

	rs2::frameset M_frames;
	rs2::frameset M_frames_newest;
	string M_timestanp;
	string M_timestanp_newest;

	std::vector<std::string> M_frame_name;
	//std::vector<rs2::video_frame> frame_vector;

	cv::Mat *M_frame0;
	cv::Mat *M_frame1;
	cv::Mat *M_frame2;
	cv::Mat *M_frame3;

	cv::Mat *M_p_img_color;
	cv::Mat *M_p_img_depth;
	cv::Mat *M_p_img_ir_left;
	cv::Mat *M_p_img_ir_right;
	cv::Mat *M_p_img_temp_8UC3;

	pcl::PointCloud<pcl::PointXYZ>::Ptr M_p_PointCloud_XYZ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_p_PointCloud_XYZRGB;


	boost::mutex M_mutex;
	boost::thread M_thread_Frames_loop;
	boost::condition M_cond;
	bool M_b_wake_thread;
	string M_time_start;
	int count_5;

	CTimeString M_time_;

	const int M_width_color = 1280;
	const int M_height_color = 720;
	const int M_width_depth = 848;
	const int M_height_depth = 480;

	const static int M_WIDTH = 1280;
	const static int M_HEIGHT = 720;
	const static int M_DEPTHMINVALUE = 105; //2700; //[mm]
	const static int M_DEPTHMAXVALUE = 10000; //2700; //[mm]
	const static int M_FOCAL_LENGTH_FY = 923912;//fx in RealSense color
	const static int M_FOCAL_LENGTH_FZ = 924449;//fy in RealSense color




	void thread_start();

public:
	//struct SDataset {
	//private:
	//	cv::Mat *p_img_color;
	//	cv::Mat *p_img_depth;
	//	cv::Mat *p_img_ir_left;
	//	cv::Mat *p_img_ir_right;
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr p_PointCloud_XYZ;
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_XYZRGB;

	//public:
	//	cv::Mat* get_img_color() { return p_img_color; }
	//	cv::Mat* get_img_depth() { return p_img_depth; }
	//	cv::Mat* get_img_ir_left() { return p_img_ir_left; }
	//	cv::Mat* get_img_ir_right() { return p_img_ir_right; }
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr get_PointCloud_XYZ() { return p_PointCloud_XYZ; }
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_PointCloud_XYZRGB() { return p_PointCloud_XYZRGB; }
	//};

	cv::Mat* get_img_color_individual(bool b_align_to_color_img = true);
	cv::Mat* get_img_depth_individual(bool b_align_to_color_img = true);
	cv::Mat* get_img_ir_left_individual(bool b_align_to_color_img = true);
	cv::Mat* get_img_ir_right_individual(bool b_align_to_color_img = true);
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_PointCloud_XYZ_individual();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_PointCloud_XYZRGB_individual();

	// Point Cloud Color Handler
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;				//ëÂè‰ïvÅH
	pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB>::Ptr m_handler;


	//SDataset M_dataset;

	void connect();
	CRealSenseInterface();

	bool updateFrames();
	void showFrame();
	void connect_thread();

	void doFrames_loop();

	void connect_ir();
	void showFrame_ir();

	void initVisualizer();
	void show_PointCloud();
	void disconnect();

};

//finish thread