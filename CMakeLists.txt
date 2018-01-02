cmake_minimum_required(VERSION 2.8.3)
project(path_following)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

include (FindPkgConfig)
pkg_check_modules (${PROJECT_NAME}_extra REQUIRED eigen3)
include_directories (${${PROJECT_NAME}_extra_INCLUDE_DIRS})

add_message_files(
  FILES
  TestMsg.msg
)
generate_messages(DEPENDENCIES std_msgs )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_following
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cc)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
