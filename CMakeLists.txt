set(OpenCV_DIR "../OCV_install")
set(CMAKE_BUILD_FILES_DIRECTORY "./build")

set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 2.8)

project( fisheye_stereo )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS}
 ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(DEWARPER_SRCS
	src/dewarper.cpp
	src/FisheyeDewarper.cpp
	src/SurroundSystem.cpp
	src/StereoPair.cpp
	src/models/CameraModel.cpp
	src/models/Scaramuzza.cpp
	src/models/Pinhole.cpp
	src/models/Atan.cpp
	src/models/Mei.cpp
	src/models/KannalaB.cpp

	include/CameraModel.h
	include/FisheyeDewarper.hpp
	include/models.h
	include/SurroundSystem.hpp
	include/StereoPair.hpp
)

add_executable( dewarper ${DEWARPER_SRCS})
target_include_directories(dewarper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


target_link_libraries( dewarper ${OpenCV_LIBS})

add_compile_options(/D_USE_MATH_DEFINES)