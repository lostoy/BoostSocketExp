cmake_minimum_required(VERSION 2.8)
project(PCL_socket_test)

set(Boost_DIR C:/local/boost_1_55_0_vs11_64/include)
set(Boost_LIBRARY_DIR C:/local/boost_1_55_0_vs11_64/lib)

find_package(Boost)
find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointCloudSocket PointCloudSocket.cpp SocketServer.cpp SocketServer.hpp )
add_library(gpu_utility SHARED gpu_utility.cpp gpu_utility.h)

target_link_libraries( PointCloudSocket ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


