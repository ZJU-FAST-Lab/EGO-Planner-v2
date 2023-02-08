cmake_minimum_required(VERSION 2.8.3)
project(selected_points_publisher)
find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Quick)
set(QT_LIBRARIES Qt5::Widgets)
qt5_wrap_cpp(MOC_FILES
  include/selected_points_publisher/selected_points_publisher.hpp
  )
add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/selected_points_publisher.cpp
  ${MOC_FILES}
  )

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
