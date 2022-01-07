cmake_minimum_required(VERSION 3.5.1)
project(vision_costmap_layer LANGUAGES C CXX VERSION ${version})

set(CMAKE_CXX_STANDARD 14)
set(CXX_STANDARD_REQUIRED ON)

# ##############################################################################
# Load and setup packages

find_package(catkin
             REQUIRED
             roscpp
             costmap_2d
             dynamic_reconfigure
             custom_msgs
             tf)

# ##############################################################################
# Generate messages and configurations
generate_dynamic_reconfigure_options(cfg/VisionLayer.cfg)

# ##############################################################################
# Setup catkin packages
catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               vision_layer
               CATKIN_DEPENDS
               costmap_2d
               dynamic_reconfigure
               custom_msgs
               tf
               DEPENDS)

# ##############################################################################
# Generate library
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(vision_layer src/vision_layer.cpp)
target_link_libraries(vision_layer ${catkin_LIBRARIES})
add_dependencies(vision_layer ${PROJECT_NAME}_gencfg)
