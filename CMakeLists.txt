cmake_minimum_required(VERSION 2.8)
project( Stereo )

find_package( OpenCV REQUIRED )

find_package(Boost COMPONENTS filesystem timer program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES} -lboost_filesystem -lboost_system)

add_executable( stereo_viewer stereo_viewer.cpp )
target_link_libraries( stereo_viewer ${OpenCV_LIBS} )

add_executable( corner_finder corner_finder.cpp )
target_link_libraries( corner_finder ${OpenCV_LIBS} )
