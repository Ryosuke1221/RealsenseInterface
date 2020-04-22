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

	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	auto color_intr = color_stream.get_intrinsics(); // Calibration data
	cout << "fx = " << color_intr.fx << endl;
	cout << "fy = " << color_intr.fy << endl;
	cout << "height = " << color_intr.height << endl;
	cout << "width = " << color_intr.width << endl;

	auto ir_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
	auto ir_intr = color_stream.get_intrinsics(); // Calibration data
	cout << "fx = " << ir_intr.fx << endl;
	cout << "fy = " << ir_intr.fy << endl;
	cout << "height = " << ir_intr.height << endl;
	cout << "width = " << ir_intr.width << endl;


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


	M_frame0 = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);
	//M_frame1_changed = new cv::Mat(cv::Size(width_color, height_color), CV_8UC3, cv::Mat::AUTO_STEP);
	//M_frame2 = new cv::Mat(cv::Size(width_depth, height_depth), CV_8UC3, cv::Mat::AUTO_STEP);
	M_frame1 = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);
	M_frame2 = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_frame3 = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);

	M_p_img_color = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);
	//M_p_img_depth = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);
	M_p_img_depth = new cv::Mat(cv::Size(M_width_color, M_width_color), CV_8UC3, cv::Mat::AUTO_STEP);
	M_p_img_ir_left = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_ir_right = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_temp_8UC3 = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3, cv::Mat::AUTO_STEP);

	M_p_PointCloud_XYZ = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared();
	M_p_PointCloud_XYZRGB = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

	M_frame_name.emplace_back("COLOR");
	M_frame_name.emplace_back("DEPTH");
	M_frame_name.emplace_back("IR1_left");
	M_frame_name.emplace_back("IR2_right");

	////show only
	//cv::namedWindow(M_frame_name[0], cv::WINDOW_NORMAL);
	//cv::namedWindow(M_frame_name[1], cv::WINDOW_NORMAL);
	//cv::namedWindow(M_frame_name[2], cv::WINDOW_NORMAL);
	//cv::namedWindow(M_frame_name[3], cv::WINDOW_NORMAL);

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
	Sleep(20);

	//Sleep(2 * 1000);
	boost::mutex::scoped_lock lock(M_mutex);
	//boost::unique_lock<boost::mutex> lock(M_mutex);

	cout << "getFrames_loop()" << endl;
	M_frames = M_pipe.wait_for_frames();
	M_timestanp = M_time_.getTimeString();
	//Sleep(10);
	//Sleep(100);

	if (count_5 == 4) {
		cout << "wait start" << endl;
		M_cond.wait(lock, [&] {return M_b_wake_thread; });
		//M_cond.wait(lock, [=] {return M_b_wake_thread; });
		cout << "wait end" << endl;
		M_b_wake_thread = false;
	}
	//M_b_wake_thread = true;
	//M_cond.notify_all();

	count_5++;
	if (count_5 == 5) count_5 = 0;
}

bool CRealSenseInterface::updateFrames()
{
	//updata data from M_frames;
	bool b_use_try_lock = true;
	//b_use_try_lock = false;
	bool b_result = false;

	if (b_use_try_lock)
	{
		boost::mutex::scoped_try_lock lock(M_mutex);
		if (lock.owns_lock())
		{
			cout << "frames updated" << endl;
			M_frames_newest = M_frames;
			M_timestanp_newest = M_timestanp;
			b_result =  true;
		}

		else b_result = false;

	}

	else
	{
		boost::mutex::scoped_lock lock(M_mutex);

		cout << "frames updated" << endl;
		M_frames_newest = M_frames;
		M_timestanp_newest = M_timestanp;

		b_result = true;
	}

	//M_b_wake_thread = true;
	//M_cond.notify_all();

	return b_result;
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

	bool b_show_cout = true;
	b_show_cout = false;

	while (!m_viewer->wasStopped())
	{
		cout << "show loop" << endl;

		if (b_show_cout)
			cout << "t1 = " << M_time_.getTimeString() << endl;

		if (updateFrames())
			get_PointCloud_XYZRGB_individual();

		if (b_show_cout)
			cout << "t2 = " << M_time_.getTimeString() << endl;

		// Update Viewer
		m_viewer->spinOnce();

		if (b_show_cout)
			cout << "t3 = " << M_time_.getTimeString() << endl;


		//cout << "show" << endl;
		if (M_p_PointCloud_XYZRGB)
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


cv::Mat* CRealSenseInterface::get_img_color_individual(bool b_align_to_color_img)
{
	//cout << "get color image" << endl;

	rs2::video_frame frame_data = M_frames_newest.get_color_frame();

	if (b_align_to_color_img)
	{
		//https://qiita.com/idev_jp/items/3eba792279d836646664
		//solve the probrem that the angles of color and depth is diffefent;
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(M_frames_newest);
		frame_data = aligned_frames.get_color_frame();
	}

	*M_p_img_color = cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3,
		(void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();

	return M_p_img_color;
}

cv::Mat* CRealSenseInterface::get_img_depth_individual(bool b_align_to_color_img)
{
	//cout << "get depth image" << endl;
	
	rs2::video_frame frame_data = M_frames_newest.get_depth_frame().apply_filter(M_colorizer);

	//cout << "frame_data size = " << frame_data.get_data_size() << endl;
	//cout << "frame_data height = " << frame_data.get_height() << endl;
	//cout << "frame_data width = " << frame_data.get_width() << endl;

	//frame_data.get_data

	if (b_align_to_color_img)
	{
		//https://qiita.com/idev_jp/items/3eba792279d836646664
		//solve the probrem that the angles of color and depth is diffefent;
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(M_frames_newest);

		frame_data = aligned_frames.get_depth_frame().apply_filter(M_colorizer);

		//cout << "frame_data size = " << frame_data.get_data_size() << endl;
		//cout << "frame_data height = " << frame_data.get_height() << endl;
		//cout << "frame_data width = " << frame_data.get_width() << endl;
	}

	*M_p_img_depth = cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC3,
		(void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();

	return M_p_img_depth;
}

cv::Mat* CRealSenseInterface::get_img_ir_left_individual(bool b_align_to_color_img)
{
	//cout << "get ir left image" << endl;

	rs2::video_frame frame_data = M_frames_newest.get_infrared_frame(2);

	//don't work
	if (b_align_to_color_img)
	{
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(M_frames_newest);
		frame_data = aligned_frames.get_infrared_frame(1);
	}

	*M_p_img_ir_left =
		cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, (void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();

	return M_p_img_ir_left;
}

cv::Mat* CRealSenseInterface::get_img_ir_right_individual(bool b_align_to_color_img)
{
	//cout << "get ir right image" << endl;

	rs2::video_frame frame_data = M_frames_newest.get_infrared_frame(2);

	//don't work
	if (b_align_to_color_img)
	{
		rs2::align align(RS2_STREAM_COLOR);
		auto aligned_frames = align.process(M_frames_newest);
		frame_data = aligned_frames.get_infrared_frame(2);
	}

	*M_p_img_ir_right = 
		cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, (void*)frame_data.get_data(), cv::Mat::AUTO_STEP).clone();

	return M_p_img_ir_right;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CRealSenseInterface::get_PointCloud_XYZ_individual()
{
	//under construction
	return M_p_PointCloud_XYZ;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CRealSenseInterface::get_PointCloud_XYZRGB_individual()
{
	cout << "get_PointCloud_XYZRGB_individual" << endl;

	//image copy ok ?
	get_img_color_individual();
	get_img_depth_individual();
	//get_img_color_individual(false);
	//get_img_depth_individual(false);

	////range
	//{
	//	for (int v = 0; v < 720; v++)
	//		for (int u = 0; u < 1280; u++)
	//		{
	//			if (v % 100 == 0 && u % 100 == 0)
	//			{
	//				//https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats
	//				float pixel_distance_in_meters = M_frames.get_depth_frame().get_distance(u, v);
	//				cout << "u:" << u << " v:" << v;
	//				cout << " depth value = " << pixel_distance_in_meters << endl;
	//			}
	//		}
	//}

	union UDEPTH
	{
		int value;
		BYTE data[3];
	};

	int WIDTH = M_WIDTH;
	int HEIGHT = M_HEIGHT;

	// allocate point cloud
	int u, v, n_pixels = WIDTH * HEIGHT;
	M_p_PointCloud_XYZRGB->clear();
	M_p_PointCloud_XYZRGB->width = WIDTH;
	M_p_PointCloud_XYZRGB->height = HEIGHT;
	M_p_PointCloud_XYZRGB->points.resize(WIDTH * HEIGHT);
	M_p_PointCloud_XYZRGB->is_dense = false;
	UDEPTH fbdepth;
	fbdepth.value = 0;
	int idxShift = 0; int idx = 0;
	double x, y, z;
	M_p_PointCloud_XYZ->clear();
	M_p_PointCloud_XYZ->points.resize(WIDTH);
	int nRangeDataIDx = 0;
	double dist = 0.0;

	for (int j = 0; j < HEIGHT; ++j)
	{
		idxShift = j * WIDTH;

		for (int i = 0; i < WIDTH; ++i)
		{
			idx = idxShift + i;
			// read depth data
			fbdepth.data[0] = M_p_img_depth->data[idx * 3];
			fbdepth.data[1] = M_p_img_depth->data[idx * 3 + 1];
			fbdepth.data[2] = M_p_img_depth->data[idx * 3 + 2];

			// compute X, Y coordinates
			u = idx % WIDTH - WIDTH / 2;
			v = HEIGHT / 2 - j;

			x = fbdepth.value / 1000.0f;
			y = -(fbdepth.value / 1000.0f) / (M_FOCAL_LENGTH_FY * 0.001f) * u / 1.0f;
			z = (fbdepth.value / 1000.0f) / (M_FOCAL_LENGTH_FZ * 0.001f) * v / 1.0f; 

			if (fbdepth.value < M_DEPTHMINVALUE || fbdepth.value > M_DEPTHMAXVALUE)
			{
				M_p_PointCloud_XYZRGB->points[idx].x = NULL;
				M_p_PointCloud_XYZRGB->points[idx].y = NULL;
				M_p_PointCloud_XYZRGB->points[idx].z = NULL;
			}
			else
			{
				M_p_PointCloud_XYZRGB->points[idx].x = x;
				M_p_PointCloud_XYZRGB->points[idx].y = y;
				M_p_PointCloud_XYZRGB->points[idx].z = z;

				//M_p_PointCloud_XYZRGB->points[idx].b = M_p_img_color->data[((j * WIDTH) + (i)) * 3];
				//M_p_PointCloud_XYZRGB->points[idx].g = M_p_img_color->data[((j * WIDTH) + (i)) * 3 + 1];
				//M_p_PointCloud_XYZRGB->points[idx].r = M_p_img_color->data[((j * WIDTH) + (i)) * 3 + 2];
				M_p_PointCloud_XYZRGB->points[idx].b = 255;
				M_p_PointCloud_XYZRGB->points[idx].g = 255;
				M_p_PointCloud_XYZRGB->points[idx].r = 255;
			}
		}
	}
	idxShift = 0;
	
	return M_p_PointCloud_XYZRGB;
}

void CRealSenseInterface::showFrame()
{
	if (updateFrames())
	{
		get_img_color_individual();
		get_img_depth_individual();
		get_img_ir_left_individual();
		get_img_ir_right_individual();
		//get_img_color_individual(false);
		//get_img_depth_individual(false);
		//get_img_ir_left_individual(false);
		//get_img_ir_right_individual(false);
	}

	cv::waitKey(1);
	
	//https://hidehiroqt.com/archives/173
	double ratio_show;
	//ratio_show = 1.;
	ratio_show = 0.5;
	//cv::resizeWindow(M_frame_name[0], M_frame0->cols* ratio_show, M_frame0->rows* ratio_show);
	//cv::resizeWindow(M_frame_name[1], M_frame1->cols* ratio_show, M_frame1->rows* ratio_show);
	//cv::resizeWindow(M_frame_name[2], M_frame2->cols* ratio_show, M_frame2->rows* ratio_show);
	//cv::resizeWindow(M_frame_name[3], M_frame3->cols* ratio_show, M_frame3->rows* ratio_show);
	cv::resizeWindow(M_frame_name[0], M_p_img_color->cols* ratio_show, M_p_img_color->rows* ratio_show);
	cv::resizeWindow(M_frame_name[1], M_p_img_depth->cols* ratio_show, M_p_img_depth->rows* ratio_show);
	cv::resizeWindow(M_frame_name[2], M_p_img_ir_left->cols* ratio_show, M_p_img_ir_left->rows* ratio_show);
	cv::resizeWindow(M_frame_name[3], M_p_img_ir_right->cols* ratio_show, M_p_img_ir_right->rows* ratio_show);
	cv::imshow(M_frame_name[0], *M_p_img_color);
	cv::imshow(M_frame_name[1], *M_p_img_depth);
	cv::imshow(M_frame_name[2], *M_p_img_ir_left);
	cv::imshow(M_frame_name[3], *M_p_img_ir_right);
}

void CRealSenseInterface::showFrame_ir()
{
	if (updateFrames())
	{
		cout << "debug: 0" << endl;
		get_img_ir_left_individual();
		//get_img_ir_right_individual();
		cout << "debug: 1" << endl;

	}

	cv::waitKey(1);

	//https://hidehiroqt.com/archives/173
	double ratio_show;
	//ratio_show = 1.;
	ratio_show = 0.5;
	cv::resizeWindow(M_frame_name[2], M_p_img_ir_left->cols* ratio_show, M_p_img_ir_left->rows* ratio_show);
	//cv::resizeWindow(M_frame_name[3], M_p_img_ir_right->cols* ratio_show, M_p_img_ir_right->rows* ratio_show);

	cv::imshow(M_frame_name[2], *M_p_img_ir_left);
	//cv::imshow(M_frame_name[3], *M_p_img_ir_right);
}

void CRealSenseInterface::connect_ir()
{
	//M_cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);//left
	//M_cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);//right

	M_cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);//left

	M_p_img_ir_left = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);
	M_p_img_ir_right = new cv::Mat(cv::Size(M_width_color, M_height_color), CV_8UC1, cv::Mat::AUTO_STEP);

	M_frame_name.emplace_back("COLOR");
	M_frame_name.emplace_back("DEPTH");
	M_frame_name.emplace_back("IR1_left");
	M_frame_name.emplace_back("IR2_right");

	//show only
	//cv::namedWindow(M_frame_name[0], cv::WINDOW_NORMAL);
	//cv::namedWindow(M_frame_name[1], cv::WINDOW_NORMAL);
	cv::namedWindow(M_frame_name[2], cv::WINDOW_NORMAL);
	//cv::namedWindow(M_frame_name[3], cv::WINDOW_NORMAL);

	//M_pipe.start(M_cfg);
	auto profile = M_pipe.start(M_cfg);

	//auto ir_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
	//auto ir_intr = color_stream.get_intrinsics(); // Calibration data
	//cout << "fx = " << ir_intr.fx << endl;
	//cout << "fy = " << ir_intr.fy << endl;
	//cout << "height = " << ir_intr.height << endl;
	//cout << "width = " << ir_intr.width << endl;

}