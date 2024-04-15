cmake_minimum_required(VERSION 3.8)
project(multi_livox_merge)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -pthread")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-O0 -Wall -Wextra -g -pthread")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 1)
set(${PROJECT_NAME}_VERSION
  ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

option(BUILD_WITH_CCACHE "Build with ccache" OFF)
if(NOT MSVC AND NOT XCODE_VERSION)
    find_program(CCACHE_FOUND ccache)
    if(CCACHE_FOUND)
        message("Found ccache: ${CCACHE_FOUND}")
        if(BUILD_WITH_CCACHE)
        message("USE ccache")
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
        endif()
    endif(CCACHE_FOUND)
endif()

find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)

include_directories(
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_msgs_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
)

# for rclcpp
find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})

# for openmp
find_package(OpenMP REQUIRED)

# for pcl ${PCL_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS} ${PCL_LIBRARIES}
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(pcl_conversions REQUIRED) # ${pcl_conversions_INCLUDE_DIRS} ${pcl_conversions_LIBRARIES}
include_directories(${pcl_conversions_INCLUDE_DIRS})

# for glog
find_package(glog REQUIRED)

add_subdirectory(src)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/config
  DESTINATION backup_config
)

install(
  FILES
      ${PROJECT_SOURCE_DIR}/shell/setenv.bash
      ${PROJECT_SOURCE_DIR}/shell/launch.bash
  DESTINATION .
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version/version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/version.h @ONLY IMMEDIATE
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/version.h
  DESTINATION include
)

execute_process(
    COMMAND tar --exclude=build 
                --exclude=.gitignore 
                --exclude=.cache
                --exclude=.git
                --exclude=.tag*
                --exclude=install
                --exclude=.vscode 
                -czf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_SourceCode_v${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION}.tar.gz 
                multi_livox_merge/
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../
)

# CPack
set(CPACK_GENERATOR "ZIP")  # ZIP TGZ DEB RPM
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION_MAJOR ${${PROJECT_NAME}_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${${PROJECT_NAME}_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${${PROJECT_NAME}_PATCH_VERSION})
set(CPACK_PACKAGE_DESCRIPTION "SIASUN")

# How to install all dependencies libraries to pack
# set(CPACK_DEB_COMPONENT_INSTALL ON)
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "libglog.so.0")

include(CPack)