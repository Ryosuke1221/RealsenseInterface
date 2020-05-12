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
	rs2::frameset M_frames_onetime;
	string M_timestanp;
	string M_timestanp_onetime;

	std::vector<std::string> M_frame_name;

	cv::Mat *M_p_img_color;
	cv::Mat *M_p_img_depth;
	cv::Mat *M_p_img_depth_show;
	cv::Mat *M_p_img_ir_left;
	cv::Mat *M_p_img_ir_right;
	cv::Mat *M_p_img_temp_8UC3;

	cv::Mat *M_p_img_color_showonly;
	cv::Mat *M_p_img_depth_showonly;
	cv::Mat *M_p_img_depth_show_showonly;
	cv::Mat *M_p_img_ir_left_showonly;
	cv::Mat *M_p_img_ir_right_showonly;


	pcl::PointCloud<pcl::PointXYZ>::Ptr M_p_PointCloud_XYZ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_p_PointCloud_XYZRGB;


	boost::thread M_thread_Frames_loop;
	boost::condition M_cond;
	bool M_b_wake_thread;
	string M_time_start;
	int count_5;

	boost::mutex M_mutex_frames;
	boost::mutex M_mutex_frames_onetime;
	boost::mutex M_mutex_img_color;
	boost::mutex M_mutex_img_depth;
	boost::mutex M_mutex_img_depth_show;
	boost::mutex M_mutex_img_ir_left;
	boost::mutex M_mutex_img_ir_right;
	boost::mutex M_mutex_PC_XYZ;
	boost::mutex M_mutex_PC_XYZRGB;

	CTimeString M_time_;

	const int M_width_color = 1280;
	const int M_height_color = 720;
	const int M_width_depth = 848;
	const int M_height_depth = 480;

	const static int M_WIDTH = 1280;
	const static int M_HEIGHT = 720;
	const static int M_DEPTHMINVALUE = 105; //2700; //[mm]
	const static int M_DEPTHMAXVALUE = 10000; //2700; //[mm]
	//const static int M_FOCAL_LENGTH_FY = 923912;//fx in RealSense color
	//const static int M_FOCAL_LENGTH_FZ = 924449;//fy in RealSense color
	int M_FOCAL_LENGTH;

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

	void update_frames_onetime();

	rs2::frameset get_frames_onetime();	//img = rs_int.get_img_color_calc(rs_int.get_M_frames());
	cv::Mat* get_img_color();
	cv::Mat* get_img_depth();
	cv::Mat* get_img_depth_show();
	cv::Mat* get_img_ir_left();
	cv::Mat* get_img_ir_right();
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_PointCloud_XYZ();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_PointCloud_XYZRGB();

	cv::Mat get_img_color_calc(rs2::frameset frame, bool b_align_to_color_img = true);
	cv::Mat get_img_depth_calc(rs2::frameset frame, bool b_colorize, bool b_align_to_color_img = true);
	cv::Mat get_img_ir_left_calc(rs2::frameset frame, bool b_align_to_color_img = true);
	cv::Mat get_img_ir_right_calc(rs2::frameset frame, bool b_align_to_color_img = true);
	void get_PointCloud_XYZRGB_calc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud,
		cv::Mat *p_img_color, cv::Mat *p_img_depth);

	// Point Cloud Color Handler
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;				//ëÂè‰ïvÅH
	pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB>::Ptr m_handler;


	//SDataset M_dataset;

	void connect();
	CRealSenseInterface();

	void showFrame();
	void connect_thread();

	void initVisualizer();
	void show_PointCloud();
	void disconnect();

	void doFrames_loop();

};

//finish thread