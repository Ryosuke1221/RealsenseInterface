#include"RealSenseInterface.h"

CRealSenseInterface::CRealSenseInterface()
{
	M_b_wake_thread = false;

	M_time_start = M_time_.getTimeString();

	count_5 = 0;
}


void CRealSenseInterface::connect()
{
	//M_cfg.enable_stream(RS2_STREAM_COLOR, M_width_color, M_height_color, RS2_FORMAT_RGB8, 30);
	M_cfg.enable_stream(RS2_STREAM_COLOR, M_width_color, M_height_color, RS2_FORMAT_BGR8, 30);	//don't work?
	//M_cfg.enable_stream(RS2_STREAM_DEPTH, M_width_depth, M_height_depth, RS2_FORMAT_Z16, 30);	//error?
	M_cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
	M_cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);//left
	M_cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);//right

	//M_cfg.enable_stream(RS2_STREAM_COLOR,3, M_width_color, M_height_color, RS2_FORMAT_RGB8, 30);
	////M_cfg.enable_stream(RS2_STREAM_DEPTH, 4, M_width_depth, M_height_depth, RS2_FORMAT_Z16, 30);	//error?
	//M_cfg.enable_stream(RS2_STREAM_DEPTH, 4, M_width_color, M_height_color, RS2_FORMAT_Z16, 30);	//error?
	//M_cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);//left
	//M_cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);//right

	//M_cfg.enable_stream(RS2_STREAM_COLOR, M_width_color, M_height_color, RS2_FORMAT_RGB8, 30);
	////M_cfg.enable_stream(RS2_STREAM_DEPTH, 4, M_width_depth, M_height_depth, RS2_FORMAT_Z16, 30);	//error?
	//M_cfg.enable_stream(RS2_STREAM_DEPTH, M_width_color, M_height_color, RS2_FORMAT_Z16, 30);	//error?
	//M_cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);//left
	//M_cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);//right
	////M_cfg.enable_stream(RS2_STREAM_INFRARED, 1);//left
	////M_cfg.enable_stream(RS2_STREAM_INFRARED, 2);//right


	//M_pipe.start(M_cfg);
	auto profile = M_pipe.start(M_cfg);

	auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto depth_intr = depth_stream.get_intrinsics(); // Calibration data
	cout << "fx = " << depth_intr.fx << endl;
	cout << "fy = " << depth_intr.fy << endl;
	cout << "height = " << depth_intr.height << endl;
	cout << "width = " << depth_intr.width << endl;
	M_FOCAL_LENGTH = depth_intr.fx;

	//auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	//auto color_intr = color_stream.get_intrinsics(); // Calibration data
	//cout << "fx = " << color_intr.fx << endl;
	//cout << "fy = " << color_intr.fy << endl;
	//cout << "height = " << color_intr.height << endl;
	//cout << "width = " << color_intr.width << endl;

	//auto ir_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
	//auto ir_intr = color_stream.get_intrinsics(); // Calibration data
	//cout << "fx = " << ir_intr.fx << endl;
	//cout << "fy = " << ir_intr.fy << endl;
	//cout << "height = " << ir_intr.height << endl;
	//cout << "width = " << ir_intr.width << endl;


	////https://qiita.com/protocol1964/items/941f52297afb3353d1b7
	////auto depth_scale = profile.get_device().first_; get_depth_scale
	////auto depth_scale = depth_stream.;

	////const float scale = rs2_get_depth_scale((profile.get_device().query_sensors()[0]), NULL);

	////get intrinsic parameter(fx, fy) height, width
	////http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
	////https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/h/rs_types.h#L55
	////https://github.com/IntelRealSense/librealsense/issues/1706
	//auto const aaa = M_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
	//cout << "fx = " << aaa.fx << endl;
	//cout << "fy = " << aaa.fy << endl;
	//cout << "height = " << aaa.height << endl;
	//cout << "width = " << aaa.width << endl;

	M_p_img_color = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);
	//M_p_img_depth = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);
	M_p_img_depth = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_16UC1, cv::Mat::AUTO_STEP);

	M_p_img_depth_show = new cv::Mat(cv::Size(M_width_color, M_width_color), CV_8UC3, cv::Mat::AUTO_STEP);
	//M_p_img_depth_show = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Scalar(255, 255, 255));

	M_p_img_ir_left = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_ir_right = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_temp_8UC3 = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);

	M_p_img_color_showonly = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_depth_showonly = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_depth_show_showonly = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_ir_left_showonly = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_ir_right_showonly = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);


	M_p_PointCloud_XYZ = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared();
	M_p_PointCloud_XYZRGB = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

	M_frame_name.emplace_back("COLOR");
	M_frame_name.emplace_back("DEPTH");
	M_frame_name.emplace_back("IR1_left");
	M_frame_name.emplace_back("IR2_right");
	M_frame_name.emplace_back("DEPTH_1ch");

	//show only
	cv::namedWindow(M_frame_name[0], cv::WINDOW_NORMAL);
	cv::namedWindow(M_frame_name[1], cv::WINDOW_NORMAL);
	//cv::namedWindow(M_frame_name[2], cv::WINDOW_NORMAL);
	//cv::namedWindow(M_frame_name[3], cv::WINDOW_NORMAL);
	cv::namedWindow(M_frame_name[4], cv::WINDOW_NORMAL);

}

void CRealSenseInterface::connect_thread()
{
	connect();

	//thread start
	M_thread_Frames_loop = boost::thread(&CRealSenseInterface::thread_start, this);

}

void CRealSenseInterface::disconnect()
{

	M_thread_Frames_loop.join();

}



void CRealSenseInterface::thread_start()
{
	//this function be inserted to a thread
	while (1)
	{
		doFrames_loop();
		//Sleep(10);

		//string t1 = M_time_.getTimeString();
		//Sleep(1000 * 3 + 10);
		//string t2 = M_time_.getTimeString();
		//int t_diff = M_time_.getTimeElapsefrom2Strings_millisec(t1, t2);
		//cout << "t_diff = " << t_diff << endl;

	}
}

void CRealSenseInterface::doFrames_loop()
{
	//Sleep(20);
	//Sleep(2 * 1000);

	cout << "getFrames_loop()" << endl;
	static string t1 = M_time_.getTimeString();
	static string t2 = M_time_.getTimeString();

	t1 = M_time_.getTimeString();

	//cout << "elapsed time from last(sensor):" << M_time_.getTimeElapsefrom2Strings(t1, t2) << endl;


	int cnt_ = 0;

	M_mutex_frames.lock();

	M_frames = M_pipe.wait_for_frames();
	M_timestanp = M_time_.getTimeString();

	cout << "ir save ";
	cout << M_time_.getTimeElapsefrom2Strings(t1, M_time_.getTimeString()) << endl;
	M_mutex_img_ir_left.lock();
	*M_p_img_ir_left = get_img_ir_left_calc(M_frames, false);
	M_mutex_img_ir_left.unlock();

	M_mutex_img_ir_right.lock();
	*M_p_img_ir_right = get_img_ir_right_calc(M_frames, false);
	M_mutex_img_ir_right.unlock();

	cout << "depth_show save ";
	cout << M_time_.getTimeElapsefrom2Strings(t1, M_time_.getTimeString()) << endl;
	M_mutex_img_depth_show.lock();
	*M_p_img_depth_show = get_img_depth_calc(M_frames, true, true);	//colorize
	M_mutex_img_depth_show.unlock();

	M_mutex_img_color.lock();
	M_mutex_img_depth.lock();
	M_mutex_PC_XYZ.lock();
	M_mutex_PC_XYZRGB.lock();


	cout << "color save ";
	cout << M_time_.getTimeElapsefrom2Strings(t1, M_time_.getTimeString()) << endl;
	*M_p_img_color = get_img_color_calc(M_frames);
	cout << "depth save ";
	cout << M_time_.getTimeElapsefrom2Strings(t1, M_time_.getTimeString()) << endl;
	*M_p_img_depth = get_img_depth_calc(M_frames, false, true);		//don't colorize

	cout << "point cloud save ";
	cout << M_time_.getTimeElapsefrom2Strings(t1, M_time_.getTimeString()) << endl;
	get_PointCloud_XYZRGB_calc(M_p_PointCloud_XYZRGB,M_p_img_color, M_p_img_depth);

	M_mutex_PC_XYZRGB.unlock();
	M_mutex_PC_XYZ.unlock();
	M_mutex_img_depth.unlock();
	M_mutex_img_color.unlock();

	M_mutex_frames.unlock();

	t2 = M_time_.getTimeString();
	cout << "elapsed time(sensor):" << M_time_.getTimeElapsefrom2Strings(t1, t2) << endl;
	cout << endl;


	//Sleep(10);
	//Sleep(500);
	

	//if (count_5 == 4) {
	//	cout << "wait start" << endl;
	//	M_cond.wait(lock, [&] {return M_b_wake_thread; });
	//	//M_cond.wait(lock, [=] {return M_b_wake_thread; });
	//	cout << "wait end" << endl;
	//	M_b_wake_thread = false;
	//}
	////M_b_wake_thread = true;
	////M_cond.notify_all();

	//count_5++;
	//if (count_5 == 5) count_5 = 0;

}

void CRealSenseInterface::initVisualizer() {

	// PCL Visualizer
	m_viewer.reset(new pcl::visualization::PCLVisualizer("Velodyne Viewer"));

	m_viewer->addCoordinateSystem(3.0, "coordinate");
	m_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	m_viewer->initCameraParameters();
	m_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	//m_viewer->removeCoordinateSystem("coordinate");		//remove axis in viewer

	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>> 
		color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>());
	m_handler = color_handler;
}

void CRealSenseInterface::show_PointCloud()
{
	cout << "show pointcloud start" << endl;

	bool b_show_cout = true;
	b_show_cout = false;

	while (!m_viewer->wasStopped())
	{
		cout << "show loop" << endl;

		if (b_show_cout)
			cout << "t1 = " << M_time_.getTimeString() << endl;

		//if (updateFrames())
		//	get_PointCloud_XYZRGB_individual();

		if (b_show_cout)
			cout << "t2 = " << M_time_.getTimeString() << endl;

		// Update Viewer
		m_viewer->spinOnce();

		if (b_show_cout)
			cout << "t3 = " << M_time_.getTimeString() << endl;

		boost::mutex::scoped_try_lock lock(M_mutex_PC_XYZRGB);
		//cout << "show" << endl;
		if (lock.owns_lock() && M_p_PointCloud_XYZRGB)
		{
			// Update Point Cloud
			m_handler->setInputCloud(M_p_PointCloud_XYZRGB);
			if (!m_viewer->updatePointCloud(M_p_PointCloud_XYZRGB, *m_handler, "cloud")) {
				m_viewer->addPointCloud(M_p_PointCloud_XYZRGB, *m_handler, "cloud");
			}
		}

		if (b_show_cout)
			cout << "t4 = " << M_time_.getTimeString() << endl;


		short key_num = GetAsyncKeyState(VK_SPACE);
		if ((key_num & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}

		if (b_show_cout)
			cout << "t5 = " << M_time_.getTimeString() << endl;


	}

}

rs2::frameset CRealSenseInterface::get_frames_onetime()
{
	boost::mutex::scoped_lock(M_mutex_frames_onetime);
	return M_frames_onetime;
}

void CRealSenseInterface::update_frames_onetime()
{
	M_mutex_frames_onetime.lock();
	M_mutex_frames.lock();
	M_frames_onetime = M_frames;
	M_timestanp_onetime = M_timestanp;
	M_mutex_frames.lock();
	M_mutex_frames_onetime.lock();
}


cv::Mat* CRealSenseInterface::get_img_color()
{
	//which fast in destorying variable and return?
	boost::mutex::scoped_lock(M_mutex_img_color);
	return M_p_img_color;
}

cv::Mat CRealSenseInterface::get_img_color_calc(rs2::frameset frame, bool b_align_to_color_img)
{
	//cout << "get color image" << endl;

	rs2::video_frame frame_data = frame.get_color_frame();

	cv::Mat img_color;

	if (b_align_to_color_img)
	{
		//https://qiita.com/idev_jp/items/3eba792279d836646664
		//solve the probrem that the angles of color and depth is diffefent;
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(frame);
		frame_data = aligned_frames.get_color_frame();

	}

	//*p_img_color = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3,
	//	(void*)frame_data.get_data(), cv::Mat::AUTO_STEP)->clone();
	img_color = cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3,
		(void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();

	return img_color;
}

cv::Mat* CRealSenseInterface::get_img_depth()
{
	boost::mutex::scoped_lock(M_mutex_img_depth);
	return M_p_img_depth;
}

cv::Mat* CRealSenseInterface::get_img_depth_show()
{
	boost::mutex::scoped_lock(M_mutex_img_depth_show);
	return M_p_img_depth_show;
}

cv::Mat CRealSenseInterface::get_img_depth_calc(rs2::frameset frame, bool b_colorize, bool b_align_to_color_img)
{
	//cout << "get depth image" << endl;
	rs2::colorizer colorizer;
	
	//cout << "true = " << true << endl;
	//cout << "b_colorize = " << b_colorize << endl;

	rs2::video_frame frame_data = frame.get_depth_frame();
	//cout << "frame_data size = " << frame_data.get_data_size() << endl;
	//cout << "frame_data height = " << frame_data.get_height() << endl;
	//cout << "frame_data width = " << frame_data.get_width() << endl;
	
	if(b_colorize)
		frame_data = frame.get_depth_frame().apply_filter(colorizer);
	//cout << "frame_data size = " << frame_data.get_data_size() << endl;
	//cout << "frame_data height = " << frame_data.get_height() << endl;
	//cout << "frame_data width = " << frame_data.get_width() << endl;


	if (b_align_to_color_img)
	{
		//https://qiita.com/idev_jp/items/3eba792279d836646664
		//solve the probrem that the angles of color and depth is diffefent;
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(frame);

		if (b_colorize)
			frame_data = aligned_frames.get_depth_frame().apply_filter(colorizer);
		else
			frame_data = aligned_frames.get_depth_frame();


		//cout << "frame_data size = " << frame_data.get_data_size() << endl;
		//cout << "frame_data height = " << frame_data.get_height() << endl;
		//cout << "frame_data width = " << frame_data.get_width() << endl;
	}


	cv::Mat img_depth;

	if (b_colorize)
		img_depth = cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3,
		(void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();
	else
		img_depth = cv::Mat(cv::Size(M_width_color, M_height_color), CV_16UC1,
			const_cast<void*>(frame_data.get_data()), cv::Mat::AUTO_STEP).clone();


	//int size_pixel = img_depth.rows * img_depth.cols;
	//cout << "size_pixel = " << size_pixel << endl;

	////http://weekendproject9.hatenablog.com/entry/2018/08/07/202153
	////get distance
	//rs2::depth_frame depth_point = frame.get_depth_frame();
	//float pixel_distance_in_meters = depth_point.get_distance(M_width_color / 2, M_height_color / 2);


	//https://qiita.com/hmichu/items/0a399d9e3bbf3a2a4454
	//fast access to pixel

	return img_depth;
}

cv::Mat* CRealSenseInterface::get_img_ir_left()
{
	boost::mutex::scoped_lock(M_mutex_img_ir_left);
	return M_p_img_ir_left;
}

cv::Mat CRealSenseInterface::get_img_ir_left_calc(rs2::frameset frame, bool b_align_to_color_img)
{
	//cout << "get ir left image" << endl;

	cv::Mat img_ir_left;

	rs2::video_frame frame_data = frame.get_infrared_frame(1);

	//don't work
	if (b_align_to_color_img)
	{
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(frame);
		frame_data = aligned_frames.get_infrared_frame(1);
	}

	img_ir_left = 
		cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1,
		(void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();

	return img_ir_left;
}

cv::Mat* CRealSenseInterface::get_img_ir_right()
{
	boost::mutex::scoped_lock(M_mutex_img_ir_right);
	return M_p_img_ir_right;
}

cv::Mat CRealSenseInterface::get_img_ir_right_calc(rs2::frameset frame, bool b_align_to_color_img)
{
	//cout << "get ir right image" << endl;

	cv::Mat img_ir_right;

	rs2::video_frame frame_data = frame.get_infrared_frame(2);

	//don't work
	if (b_align_to_color_img)
	{
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(frame);
		frame_data = aligned_frames.get_infrared_frame(2);
	}

	img_ir_right =
		cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, 
		(void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();

	return img_ir_right;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CRealSenseInterface::get_PointCloud_XYZ()
{ 
	boost::mutex::scoped_lock(M_mutex_PC_XYZ);
	return M_p_PointCloud_XYZ; 
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CRealSenseInterface::get_PointCloud_XYZRGB()
{
	boost::mutex::scoped_lock(M_mutex_PC_XYZRGB);
	return M_p_PointCloud_XYZRGB;
}

void CRealSenseInterface::get_PointCloud_XYZRGB_calc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud,
	cv::Mat *p_img_color, cv::Mat *p_img_depth)
{
	int cnt_ = 0;

	if ((p_img_color->rows != p_img_depth->rows)
		|| p_img_color->cols != p_img_depth->cols)
	{
		cout << "Error: image size is different (PointCloud)" << endl;
		return ;
	}

	//range
	{
		//for (int v = 0; v < 720; v++)
		//	for (int u = 0; u < 1280; u++)
		//	{
		//		if (v % 100 == 0 && u % 100 == 0)
		//		{
		//			//https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats
		//			float pixel_distance_in_meters = M_frames.get_depth_frame().get_distance(u, v);
		//			cout << "u:" << u << " v:" << v;
		//			cout << " depth value = " << pixel_distance_in_meters << endl;
		//		}
		//	}

		//float pixel_distance_in_meters = M_frames.get_depth_frame().get_distance(p_img_depth->cols / 2, p_img_depth->rows / 2);
		//cout << "pixel_distance_in_meters = " << pixel_distance_in_meters << endl;

	}

	int WIDTH = M_WIDTH;
	int HEIGHT = M_HEIGHT;

	// allocate point cloud
	int n_pixels = WIDTH * HEIGHT;
	p_cloud->clear();
	p_cloud->width = WIDTH;
	p_cloud->height = HEIGHT;
	p_cloud->points.resize(WIDTH * HEIGHT);
	p_cloud->is_dense = false;
	int idxShift = 0; int idx = 0;
	int nRangeDataIDx = 0;
	double dist = 0.0;

	//cout << "p_img_depth->at<uchar>(1, 1) = " << p_img_depth->at<uchar>(1, 1) << endl;
	//cout << "p_img_depth->at<float>(1, 1) = " << p_img_depth->at<float>(1, 1) << endl;

	{
		//unsigned short x = p_img_depth->at<unsigned short>(p_img_depth->rows - 1, p_img_depth->cols - 1);
		//unsigned short x = p_img_depth->at<unsigned short>(p_img_depth->rows / 2, p_img_depth->cols / 2);
		//cout << "x(center):" << x << endl;
	}

	for (int j = 0; j < HEIGHT; ++j)
	{
		idxShift = j * WIDTH;
		unsigned short *p_value = p_img_depth->ptr<unsigned short>(j);

		for (int i = 0; i < WIDTH; ++i)
		{
			double x, y, z;
			int u, v;

			idx = idxShift + i;
			// compute X, Y coordinates
			u = idx % WIDTH - WIDTH / 2;
			v = HEIGHT / 2 - j;

			x = p_value[i] / 1000.0f;
			y = -(p_value[i] / 1000.0f) / (M_FOCAL_LENGTH) * u / 1.0f;
			z = (p_value[i] / 1000.0f) / (M_FOCAL_LENGTH) * v / 1.0f;

			if (p_value[i] < M_DEPTHMINVALUE || p_value[i] > M_DEPTHMAXVALUE)
			{
				p_cloud->points[idx].x = NULL;
				p_cloud->points[idx].y = NULL;
				p_cloud->points[idx].z = NULL;
			}
			else
			{
				p_cloud->points[idx].x = x;
				p_cloud->points[idx].y = y;
				p_cloud->points[idx].z = z;

				p_cloud->points[idx].b = p_img_color->data[((j * WIDTH) + (i)) * 3];
				p_cloud->points[idx].g = p_img_color->data[((j * WIDTH) + (i)) * 3 + 1];
				p_cloud->points[idx].r = p_img_color->data[((j * WIDTH) + (i)) * 3 + 2];
			}
		}
	}

	cout << "p_cloud->size() = " << p_cloud->size() << endl;

	//double x_weight = 0.;
	//double y_weight = 0.;
	//double z_weight = 0.;
	//if (p_cloud->size() != 0)
	//{
	//	for (size_t i = 0; i < p_cloud->size(); i++)
	//	{
	//		x_weight += p_cloud->points[i].x;
	//		y_weight += p_cloud->points[i].y;
	//		z_weight += p_cloud->points[i].z;
	//	}
	//	x_weight /= p_cloud->size();
	//	y_weight /= p_cloud->size();
	//	z_weight /= p_cloud->size();
	//	cout << "x_weight:" << x_weight << endl;
	//	cout << "y_weight:" << y_weight << endl;
	//	cout << "z_weight:" << z_weight << endl;
	//}

}

void CRealSenseInterface::showFrame()
{
	bool b_show = false;
	b_show = true;

	//cv::Mat img_error = cv::Mat(cv::Size(1280, 720), CV_8UC3, cv::Scalar(255, 255, 255));

	cout << "show" << endl;

	double ratio_show;
	//ratio_show = 1.;
	ratio_show = 0.5;

	{
		boost::mutex::scoped_try_lock lock(M_mutex_img_color);
		if (lock.owns_lock()) {
			//https://hidehiroqt.com/archives/173
			//cout << "show color" << endl;
			*M_p_img_color_showonly = M_p_img_color->clone();
		}
		else
		{
			if (b_show) cout << "try failed" << endl;
		}
	}

	{
		boost::mutex::scoped_try_lock lock(M_mutex_img_depth_show);
		if (lock.owns_lock()) {
			//cout << "show depth" << endl;
			*M_p_img_depth_show_showonly = M_p_img_depth_show->clone();
		}
		else
		{
			if (b_show) cout << "try failed" << endl;
		}
	}

	{
		boost::mutex::scoped_try_lock lock(M_mutex_img_depth);
		if (lock.owns_lock()) {
			//cout << "show depth" << endl;
			*M_p_img_depth_showonly = M_p_img_depth->clone();
		}
		else
		{
			if (b_show) cout << "try failed" << endl;
		}
	}


	{
		boost::mutex::scoped_try_lock lock(M_mutex_img_ir_left);
		if (lock.owns_lock()) {
			*M_p_img_ir_left_showonly = M_p_img_ir_left->clone();
		}
		else
		{
			if (b_show) cout << "try failed" << endl;
		}

	}

	{
		boost::mutex::scoped_try_lock lock(M_mutex_img_ir_right);
		if (lock.owns_lock()) {
			*M_p_img_ir_right_showonly = M_p_img_ir_right->clone();
		}
		else
		{
			if (b_show) cout << "try failed" << endl;
		}

	}

	cv::resizeWindow(M_frame_name[0], 
		M_p_img_color_showonly->cols* ratio_show, M_p_img_color_showonly->rows* ratio_show);
	cv::resizeWindow(M_frame_name[1], 
		M_p_img_depth_show_showonly->cols* ratio_show, M_p_img_depth_show_showonly->rows* ratio_show);
	cv::resizeWindow(M_frame_name[2], 
		M_p_img_ir_left_showonly->cols* ratio_show, M_p_img_ir_left_showonly->rows* ratio_show);
	cv::resizeWindow(M_frame_name[3], 
		M_p_img_ir_right_showonly->cols* ratio_show, M_p_img_ir_right_showonly->rows* ratio_show);
	cv::resizeWindow(M_frame_name[4],
		M_p_img_depth_showonly->cols* ratio_show, M_p_img_depth_showonly->rows* ratio_show);

	cv::imshow(M_frame_name[0], *M_p_img_color_showonly);
	cv::imshow(M_frame_name[1], *M_p_img_depth_show_showonly);
	//cv::imshow(M_frame_name[2], *M_p_img_ir_left_showonly);
	//cv::imshow(M_frame_name[3], *M_p_img_ir_right_showonly);
	cv::imshow(M_frame_name[4], *M_p_img_depth_showonly);

	cv::waitKey(1);
}
