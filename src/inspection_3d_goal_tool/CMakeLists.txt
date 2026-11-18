cmake_minimum_required(VERSION 3.8)
project(inspection_3d_goal_tool)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED src/goal_tool.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp geometry_msgs pluginlib rclcpp rviz_common rviz_rendering)

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY icons/ DESTINATION share/${PROJECT_NAME}/icons)

ament_package()
