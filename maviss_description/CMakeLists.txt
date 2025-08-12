cmake_minimum_required(VERSION 3.5)
project(maviss_description)

if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
endif()

## ROS Related packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    include)
	
# install the launch file	
install(DIRECTORY
        launch
        meshes
        urdf
        world
        DESTINATION share/${PROJECT_NAME}/
        )

# Install Python module
install(PROGRAMS
  	magnetometer_node/magnetometer_node.py
  	DESTINATION lib/${PROJECT_NAME}
	)

ament_package()

