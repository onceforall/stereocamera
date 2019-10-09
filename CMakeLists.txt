PROJECT(stereo_camera)


cmake_minimum_required(VERSION 3.5)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)
find_package(Qt5Widgets)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)


add_executable(stereo_camera src/get_depth.cpp src/main.cpp src/feature_extract.cpp src/adjustimg.cpp src/image_stitch.cpp src/calibration.cpp)

link_directories(${OpenCV_LIBRARY_DIRS})

target_link_libraries(stereo_camera ${OpenCV_LIBS})
target_link_libraries(stereo_camera pthread)

