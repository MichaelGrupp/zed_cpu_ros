cmake_minimum_required(VERSION 2.8.3)
project(zed_cpu_ros)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            cv_bridge
            image_transport
            camera_info_manager
            dynamic_reconfigure
        )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
        cfg/zed_cpu_ros_dynamic_reconfigure.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
)

add_executable(zed_cpu_ros src/zed_cpu_ros.cpp include/tearing_filter.hpp)
target_link_libraries(zed_cpu_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
