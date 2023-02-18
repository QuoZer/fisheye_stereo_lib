set(OpenCV_DIR "C:/Users/Matvey/Soft/OpenCV/opencv/build")
set(CMAKE_BUILD_FILES_DIRECTORY "./build")

set (CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 3.2)

project( fisheye_stereo )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS}
 ${CMAKE_CURRENT_SOURCE_DIR}/include
 D:/Work/Coding/Repos/cv-plot/CvPlot/inc
)

# taken from: https://stackoverflow.com/questions/14306642/adding-multiple-executables-in-cmake

file( GLOB LIB_SOURCES src/*.cpp
				 src/models/*.cpp)
file( GLOB LIB_HEADERS include/*.h )
add_library( fisheye_svs ${LIB_SOURCES} ${LIB_HEADERS} )

file( GLOB APP_SOURCES example/*.cpp )
foreach( testsourcefile ${APP_SOURCES} )
    # I used a simple string replace, to cut off .cpp.
    #string( REPLACE ".cpp" "" testname ${testsourcefile} )
    get_filename_component(EXECUTABLE_NAME ${testsourcefile} NAME_WE)
    add_executable( ${EXECUTABLE_NAME} ${testsourcefile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${EXECUTABLE_NAME} fisheye_svs )
    target_link_libraries( ${EXECUTABLE_NAME} ${OpenCV_LIBS} )
endforeach( testsourcefile ${APP_SOURCES} )

#set(DEWARPER_SRCS
#	example/test.cpp
#
#	src/dewarper.cpp
#	src/FisheyeDewarper.cpp
#	src/SurroundSystem.cpp
#	src/StereoPair.cpp
#	src/Utils.cpp
#	src/models/CameraModel.cpp
#	src/models/Scaramuzza.cpp
#	src/models/Pinhole.cpp
#	src/models/Atan.cpp
#	src/models/RealAtan.cpp
#	src/models/Mei.cpp
#	src/models/KannalaB.cpp
#	src/models/DoubleSphere.cpp
#
#	include/CameraModel.h
#	include/FisheyeDewarper.hpp
#	include/models.h
#	include/SurroundSystem.hpp
#	include/StereoPair.hpp
#	include/Utils.h
#)

#add_executable( dewarper ${DEWARPER_SRCS})
#target_include_directories(dewarper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
#target_link_libraries( dewarper ${OpenCV_LIBS})
#
#add_executable( test ${DEWARPER_SRCS})
#target_include_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
#target_link_libraries( test ${OpenCV_LIBS})

add_compile_options(/D_USE_MATH_DEFINES)