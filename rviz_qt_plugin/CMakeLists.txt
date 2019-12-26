cmake_minimum_required(VERSION 2.8.3)
project(rviz_qt)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp REQUIRED
  rviz REQUIRED
  std_msgs REQUIRED
  pluginlib REQUIRED
)


catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Network Concurrent Widgets REQUIRED)
        
set(QT_LIBRARIES Qt5::Widgets)


qt5_wrap_ui(UIC_FILES src/ui/rvizqt.ui)


add_definitions(-DQT_NO_KEYWORDS)

include_directories(
 include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)



set(HEADERS
  include/rvizqt.h
)

set(SOURCE_FILE
  src/rvizqt.cpp
)

set(PROJECT_NAME rviz_qt)
add_library(${PROJECT_NAME} ${SOURCE_FILE} ${UIC_FILES} ${HEADERS} )

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(FILES
#  rivz_leo_test_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

