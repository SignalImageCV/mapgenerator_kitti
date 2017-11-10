cmake_minimum_required(VERSION 2.8.3)
project(globalmap_generator)

#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS 
roscpp rospy std_msgs sensor_msgs
cv_bridge
pcl_conversions
pcl_msgs
pcl_ros
tf
tf_conversions)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(G2O REQUIRED)
#find_package(PCL 1.7 REQUIRED)

## Declare a catkin package
catkin_package(DEPENDS OpenCV Eigen3
               INCLUDE_DIRS include
               LIBRARIES globalmap_generator
               CATKIN_DEPENDS roscpp  
               std_msgs sensor_msgs geometry_msgs
               cv_bridge
               pcl_ros pcl_conversions pcl_msgs tf tf_conversions)


###########
## Build ##
###########

set(CMAKE_BUILD_TYPE Release)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
#    SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS}
)
#include_directories(SYSTEM BEFORE ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})

set (SRC
  src/main.cpp
  src/MapGenerator.cpp
  src/MapPublisher.cpp
)

#link_directories(${G2O_LIBRARY_DIRS})

add_executable(globalmap_generator ${SRC})
target_link_libraries(globalmap_generator 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS}
#                      ${G2O_LIBRARIES}
                      ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)

add_dependencies(globalmap_generator ${catkin_EXPORTED_TARGETS})
