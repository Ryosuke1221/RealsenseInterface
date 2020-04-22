cmake_minimum_required(VERSION 2.8)

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "test_realsense" )

add_executable(
	test_realsense 
	main.cpp 
	RealSenseInterface.h RealSenseInterface.cpp 
	TimeString.h TimeString.cpp )
#add_executable(test_realsense rs-capture.cpp)

#OpenCV 3.4.1
find_package( OpenCV REQUIRED PATHS "C:/Program Files/OpenCV-3.4.1")
include_directories(${OpenCV_INCLUDE_DIRS} )
target_link_libraries(test_realsense ${OpenCV_LIBS} )

#PCL 1.8
find_package( PCL 1.8 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries( test_realsense ${PCL_LIBRARIES} )
endif()

include_directories("C:/Program Files (x86)/Intel RealSense SDK 2.0/include" )
set(LIB_FILES 
	"C:/Program Files/RealSense2/lib/x64/realsense2.lib"
)
target_link_libraries(test_realsense ${LIB_FILES} )

set(RESOURCE_FILES "C:/Program Files/RealSense2/bin/x64/realsense2.dll")

file(COPY ${RESOURCE_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
