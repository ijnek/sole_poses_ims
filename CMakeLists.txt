cmake_minimum_required(VERSION 3.8)
project(sole_poses_ims)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp_components REQUIRED)

# Build
add_library(${PROJECT_NAME}_node SHARED
  src/sole_poses_ims.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "sole_poses_ims::SolePosesIms"
  EXECUTABLE ${PROJECT_NAME})

target_link_libraries(sole_poses_ims_node PUBLIC
  ${biped_interfaces_TARGETS}
  interactive_markers::interactive_markers
  rclcpp_components::component)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  biped_interfaces
  interactive_markers
  rclcpp_components)

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
