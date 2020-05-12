#include <iostream>
#include"RealSenseInterface.h"

int maint2() try
{
	

	return EXIT_SUCCESS;
	//return 0;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}



int maint() try
{

	CRealSenseInterface rs_int;
	//rs_int.connect();
	rs_int.connect_thread();
	rs_int.initVisualizer();
	//Sleep(0.5 * 1000);

	rs_int.show_PointCloud();
	rs_int.disconnect();

	return EXIT_SUCCESS;
	//return 0;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

int main() try
{

	CRealSenseInterface rs_int;
	rs_int.initVisualizer();
	//rs_int.connect();
	rs_int.connect_thread();

	//Sleep(0.5 * 1000);

	rs_int.show_PointCloud();

	//Sleep(30 * 1000);


	//while (1)
	//{
	//	//rs_int.doFrames_loop();
	//	rs_int.showFrame();
	//	//rs_int.showFrame_test();
	//}

	rs_int.disconnect();

	return EXIT_SUCCESS;
	//return 0;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}


