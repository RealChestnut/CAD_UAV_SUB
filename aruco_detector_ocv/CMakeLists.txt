cmake_minimum_required(VERSION 3.0.2)
project(aruco_detector_ocv)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(OpenCV_DIR /home/cad-uav2/installation/OpenCV-3.4.4/installation/OpenCV-3.4.4/share/OpenCV/)

set(OpenCL_INCLUDE_DIRS "/usr/include")
set(OpenCL_LIBRARIES "/usr/lib/x86_64-linux-gnu/libOpenCL.so.1")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        image_geometry
	cv_bridge
	geometry_msgs 
	roscpp rospy 
	std_msgs 
	tf2 
	tf2_ros 
	image_transport 
	std_msgs 
	alfons_msgs
	)

if(OpenCL_FOUND)
    include_directories(${OpenCL_INCLUDE_DIRS})
    link_directories(${OpenCL_LIBRARY_DIRS})
    set(OPENCL_LIBRARIES ${OpenCL_LIBRARIES})
endif()


find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)        

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS cv_bridge image_geometry geometry_msgs std_msgs roscpp rospy std_msgs tf2 tf2_ros image_transport alfons_msgs
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${realsense_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCL_INCLUDE_DIRS}
  
)
link_directories(${OpenCL_LIBRARY_DIRS})
    

set(aruco_source    
        src/aruco_detector_ocv.cpp
       
)
    
add_executable(aruco_detector_ocv ${aruco_source})
target_link_libraries(aruco_detector_ocv ${catkin_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(ros_realsense_opencv_tutorial src/ros_realsense_opencv_tutorial.cpp)
target_link_libraries(ros_realsense_opencv_tutorial ${catkin_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY} ${OpenCL_LIBRARIES})
#set_target_properties(aruco_detector_ocv PROPERTIES OUTPUT_NAME node PREFIX "")

add_executable(camera_cal src/camera_cal.cpp)
target_link_libraries(camera_cal ${catkin_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#        launch/detector.launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )

