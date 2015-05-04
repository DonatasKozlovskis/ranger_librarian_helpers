cmake_minimum_required(VERSION 2.8.3)
project(ranger_librarian)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED regex)

# Find tesseract
find_path(Tesseract_INCLUDE_DIR tesseract/baseapi.h
  HINTS
  /usr/include
  /usr/local/include)

find_library(Tesseract_LIBRARIES NAMES tesseract_full tesseract_api tesseract
  HINTS
  /usr/lib
  /usr/local/lib)
  
  
  
# Find all required modules for compilation
# catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	roscpp
	rospy
	image_transport
	actionlib
	std_msgs
	sensor_msgs
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
	FILES
	WeightFiltered.msg
)
# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES LabelReading.action
)
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
# runtime dependencies
catkin_package(
        CATKIN_DEPENDS message_runtime actionlib_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)


#add_executable( minimal_ocr minimal_ocr.cpp)
#target_link_libraries( minimal_ocr ${OpenCV_LIBS} ${Tesseract_LIBRARIES} ${Boost_LIBRARIES})

#add_executable( video_ocr video_ocr.cpp include/utils.cpp)
#target_link_libraries( video_ocr ${OpenCV_LIBS} ${Tesseract_LIBRARIES} ${Boost_LIBRARIES})

add_executable( main src/main.cpp src/label_reader.cpp src/ranger_librarian.cpp)
target_link_libraries( main ${catkin_LIBRARIES} ${Tesseract_LIBRARIES} )

add_executable( depth_below_timer src/depth_below_timer.cpp)
target_link_libraries( depth_below_timer ${catkin_LIBRARIES} )

add_executable( weight_change_monitor src/weight_change_monitor.cpp)
target_link_libraries( weight_change_monitor ${catkin_LIBRARIES} )

add_executable( user_view_creator src/user_view_creator.cpp)
target_link_libraries( user_view_creator ${catkin_LIBRARIES} )

add_executable(label_reader_server src/label_reader_server.cpp src/label_reader.cpp)
target_link_libraries(label_reader_server ${catkin_LIBRARIES} ${Tesseract_LIBRARIES})


### install

install(
    TARGETS main depth_below_timer weight_change_monitor label_reader_server
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )



# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
