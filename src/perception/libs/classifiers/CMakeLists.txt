cmake_minimum_required(VERSION 2.8.3)
project(classifiers_lib)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS core
    roscpp
    # ROS messages
    # message_generation
    # std_msgs
    # sensor_msgs
    # ROS PCL
    # pcl_conversions
    # pcl_ros
    # ROS tf
    # tf
    # tf_conversions
    # 3rd modules for ROS, like Eigen
    cmake_modules
    # local packages
    common_lib
    roi_filters_lib
    object_builders_lib
    perception_msgs
)

## System dependencies are found with CMake's conventions
### PCL
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
    surface
    filters
    features
    segmentation
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# OpenCV
find_package(OpenCV 4.2.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

### Eigen
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS roscpp common_lib roi_filters_lib object_builders_lib perception_msgs
    #  DEPENDS system_lib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

## http://mariobadr.com/creating-a-header-only-library-with-cmake.html
#add_library(${PROJECT_NAME} INTERFACE)
#target_include_directories(${PROJECT_NAME} INTERFACE include/common)

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${Eigen3_LIBRARIES}
#)

## Declare a C++ executable
add_library(${PROJECT_NAME}
    src/ism_classifier.cpp
    src/visual_classifier.cpp
)

add_dependencies(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

#############
## Install ##
#############

# ...