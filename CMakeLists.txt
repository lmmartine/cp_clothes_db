cmake_minimum_required(VERSION 2.8.3)
project(cp_clothes_db)


## - - find packages - - 
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  roslib 
  std_msgs 
  std_srvs 
  sensor_msgs 
  uchile_msgs 
  uchile_srvs 
  image_transport 
  cv_bridge
  uchile_util
  pcl_ros
  pcl_conversions
)

find_package(OpenCV  REQUIRED)
find_package(Boost 1.4.0 COMPONENTS filesystem REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(VERSION_FILE_PATH "${CATKIN_DEVEL_PREFIX}/include")
file(MAKE_DIRECTORY "${VERSION_FILE_PATH}")


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs std_srvs sensor_msgs uchile_msgs uchile_srvs uchile_util image_transport cv_bridge pcl_ros
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  ${FLANN_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(cp_clothes_db src/cp_clothes_db.cpp )
add_dependencies(cp_clothes_db ${catkin_EXPORTED_TARGETS})
target_link_libraries(cp_clothes_db ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})