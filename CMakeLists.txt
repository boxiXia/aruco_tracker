cmake_minimum_required(VERSION 3.1)
project(aruco_tracker VERSION 3.1.0 LANGUAGES CXX C)

add_compile_options("/std:c++latest")

find_package(OpenCV REQUIRED)
# # find all opengl packages
# find_package(glm CONFIG REQUIRED)
# find_package(GLEW REQUIRED)
# find_package(glfw3 CONFIG REQUIRED)
# find_package(imgui CONFIG REQUIRED) # GUI
# find_package(msgpack CONFIG REQUIRED)
# find_package(clipp CONFIG REQUIRED)
#find_package(asio CONFIG REQUIRED)
#find_package(ghc_filesystem CONFIG REQUIRED)

set(USE_OWN_EIGEN3 off)

#include_directories(aruco)
#include_directories(aruco/src)
#include_directories(aruco/utils_calibration)
#add_subdirectory(aruco)
find_package(aruco CONFIG REQUIRED)

include_directories(marker_mapper)
#include_directories(marker_mapper/src)
#include_directories(marker_mapper/utils)
add_subdirectory(marker_mapper)


add_executable(app app.cpp)
target_link_libraries(app aruco ${OpenCV_LIBRARIES} ${THREADLIB})


add_executable(app_calibration app_calibration.cpp 
				aruco/utils_calibration/calibrator.h
				aruco/utils_calibration/calibrator.cpp)
target_link_libraries(app_calibration aruco ${OpenCV_LIBRARIES} ${THREADLIB})