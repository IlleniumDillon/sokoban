cmake_minimum_required(VERSION 3.15)
project(sokoban)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${matplotlib_cpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(sokoban main.cpp)
target_include_directories(sokoban PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sokoban Python3::Python Python3::NumPy ${OpenCV_LIBS})
