cmake_minimum_required(VERSION 3.0.2)
project(mbf_experimental_nav)

find_package(catkin REQUIRED
  COMPONENTS
  # costmap_2d
  # dynamic_reconfigure
  # geometry_msgs
  mbf_abstract_nav
  mbf_abstract_core
  pluginlib
  # mbf_msgs
  # mbf_utility
  # nav_core
  # nav_msgs
  roscpp
  # std_msgs
  # std_srvs
  # tf
)

find_package(Boost COMPONENTS thread chrono REQUIRED)

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#   cfg/MoveBaseFlex.cfg
# )

# set(MBF_NAV_CORE_WRAPPER_LIB mbf_nav_core_wrapper)
set(MBF_EXPERIMENTAL_SERVER_LIB mbf_experimental_server)
set(MBF_EXPERIMENTAL_SERVER_NODE mbf_experimental_nav)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${MBF_EXPERIMENTAL_SERVER_LIB}
  CATKIN_DEPENDS
  # actionlib
  # actionlib_msgs
  # costmap_2d
  # dynamic_reconfigure
  # geometry_msgs
  mbf_abstract_nav
  mbf_abstract_core
  # mbf_msgs
  # mbf_utility
  # nav_core
  # nav_msgs
  pluginlib
  roscpp
  # std_msgs
  # std_srvs
  # tf
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# add_library(${MBF_NAV_CORE_WRAPPER_LIB}
#   src/nav_core_wrapper/wrapper_global_planner.cpp
#   src/nav_core_wrapper/wrapper_local_planner.cpp
#   src/nav_core_wrapper/wrapper_recovery_behavior.cpp
# )
# add_dependencies(${MBF_NAV_CORE_WRAPPER_LIB} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${MBF_NAV_CORE_WRAPPER_LIB}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

add_library(${MBF_EXPERIMENTAL_SERVER_LIB}
  src/experimental_navigation_server.cpp
#   src/mbf_costmap_nav/costmap_planner_execution.cpp
#   src/mbf_costmap_nav/costmap_controller_execution.cpp
#   src/mbf_costmap_nav/costmap_recovery_execution.cpp
#   src/mbf_costmap_nav/costmap_wrapper.cpp
#   src/mbf_costmap_nav/footprint_helper.cpp
)

add_dependencies(${MBF_EXPERIMENTAL_SERVER_LIB} ${catkin_EXPORTED_TARGETS})
# add_dependencies(${MBF_COSTMAP_2D_SERVER_LIB} ${MBF_NAV_CORE_WRAPPER_LIB})
# add_dependencies(${MBF_EXPERIMENTAL_SERVER_LIB} ${PROJECT_NAME}_gencfg)

target_link_libraries(${MBF_EXPERIMENTAL_SERVER_LIB}
  # ${MBF_NAV_CORE_WRAPPER_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${MBF_EXPERIMENTAL_SERVER_NODE} src/move_base_server_node.cpp)
add_dependencies(${MBF_EXPERIMENTAL_SERVER_NODE} ${MBF_EXPERIMENTAL_SERVER_LIB})
target_link_libraries(${MBF_EXPERIMENTAL_SERVER_NODE}
  ${MBF_EXPERIMENTAL_SERVER_LIB}
  ${catkin_LIBRARIES}
)

install(TARGETS
  # ${MBF_NAV_CORE_WRAPPER_LIB} ${MBF_EXPERIMENTAL_SERVER_LIB} ${MBF_EXPERIMENTAL_SERVER_NODE}
  ${MBF_EXPERIMENTAL_SERVER_LIB} ${MBF_EXPERIMENTAL_SERVER_NODE}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# catkin_install_python(PROGRAMS scripts/move_base_legacy_relay.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
