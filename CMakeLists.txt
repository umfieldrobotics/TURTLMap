cmake_minimum_required(VERSION 2.8.3)
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.0.2")
  if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
  endif (POLICY CMP0048)
endif()
project(turtlmap)
set(CMAKE_CXX_STANDARD 14)


find_package(catkin_simple REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  waterlinked_a50_ros_driver
  voxblox_ros
)

find_package(Eigen3 REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
find_package(GTSAM REQUIRED) # Uses installed package
find_package(catkin REQUIRED rosbag)
find_package(yaml-cpp REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  # Include files are just here for IDEs
  include/TURTLMap/PosegraphBackendOnline.h
  include/TURTLMap/Posegraph.h
  include/TURTLMap/PoseGraphParameters.h
  include/TURTLMap/BluerovBarometerFactor.h
  include/TURTLMap/VelocityIntegrationFactor.h

  src/Posegraph.cpp
  src/PosegraphBackendOnline.cpp
  src/VelocityIntegrationFactor.cpp # NOTE: Do not include the executable here, it will be linked in the executable
  src/BluerovBarometerFactor.cpp # NOTE: Do not include the executable here, it will be linked in the executable
)
target_link_libraries(${PROJECT_NAME}
    gtsam
    yaml-cpp
    gtsam_unstable
    # Boost::boost
    # Boost::serialization
    # Boost::system
    # Boost::filesystem
    # Boost::thread
    # Boost::date_time
    # Boost::regex
    # Boost::timer
    # Boost::chrono
    # ${catkin_LIBRARIES}
    # ${EIGEN3_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

cs_add_executable(${PROJECT_NAME}_node
  src/PosegraphBackendOnlineNode.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


cs_install()

cs_export()
