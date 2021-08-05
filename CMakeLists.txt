cmake_minimum_required(VERSION 3.1)
project(aruco_tracker VERSION 3.1.0 LANGUAGES CXX C)

add_compile_options("/std:c++latest")

set(CMAKE_CXX_STANDARD 17) # set c++ standard to 17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # https://cmake.org/cmake/help/git-stage/prop_tgt/CXX_EXTENSIONS.html
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # If enabled, generates a compile_commands.json file of compiler calls


find_package(OpenCV REQUIRED)
# # find all opengl packages
# find_package(glm CONFIG REQUIRED)
# find_package(GLEW REQUIRED)
# find_package(glfw3 CONFIG REQUIRED)
# find_package(imgui CONFIG REQUIRED) # GUI
find_package(msgpack CONFIG REQUIRED)
find_package(clipp CONFIG REQUIRED)
#find_package(asio CONFIG REQUIRED)
#find_package(ghc_filesystem CONFIG REQUIRED)


find_package(cppzmq CONFIG REQUIRED) # for networking

find_package(asio CONFIG REQUIRED)

set(USE_OWN_EIGEN3 off)

#include_directories(aruco)
#include_directories(aruco/src)
include_directories(aruco/utils_calibration)
include_directories(aruco/utils_markermap)
#add_subdirectory(aruco)

find_package(aruco CONFIG REQUIRED)

#include_directories(aruco/utils_markermap)

#include_directories(marker_mapper)
#include_directories(marker_mapper/src)
#include_directories(marker_mapper/utils)
#add_subdirectory(marker_mapper)


add_executable(app app.cpp)
target_link_libraries(app aruco clipp::clipp ${OpenCV_LIBRARIES} ${THREADLIB})


add_executable(app_calibration app_calibration.cpp 
				aruco/utils_calibration/calibrator.h
				aruco/utils_calibration/calibrator.cpp)
target_link_libraries(app_calibration aruco ${OpenCV_LIBRARIES} ${THREADLIB})

add_executable(app_test_markermap 
		aruco_test_markermap.cpp 
		aruco/utils_markermap/pcdwriter.cpp 
		aruco/utils_markermap/sglviewer.h aruco/utils_markermap/sgl.h)
target_link_libraries(app_test_markermap aruco clipp::clipp ${OpenCV_LIBRARIES} ${THREADLIB} asio asio::asio msgpackc msgpackc-cxx)


add_executable(aruco_print_dictionary aruco_print_dictionary.cpp)
target_link_libraries(aruco_print_dictionary aruco clipp::clipp ${OpenCV_LIBRARIES} ${THREADLIB})



add_executable(       zmq_publisher  publisher.cpp  message.hpp)
target_link_libraries(zmq_publisher  cppzmq cppzmq-static msgpackc msgpackc-cxx)

add_executable(       zmq_subscriber subscriber.cpp message.hpp)
target_link_libraries(zmq_subscriber cppzmq cppzmq-static msgpackc msgpackc-cxx)

add_executable(testasio testasio.cpp)
target_link_libraries(testasio PRIVATE asio asio::asio msgpackc msgpackc-cxx)
