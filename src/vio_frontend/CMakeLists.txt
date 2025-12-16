cmake_minimum_required(VERSION 3.8)
project(vio_frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Feature.msg"
  "msg/VIOUpdate.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(${PROJECT_NAME}_lib SHARED
  src/feature_tracker.cpp
  src/feature_tracker_ros.cpp
  src/imu_preintegrator.cpp
  src/imu_preintegrator_ros.cpp
  src/vio_frontend_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  Eigen3
  OpenCV
  geometry_msgs
)

rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "vio_frontend::VioFrontendNode"
  EXECUTABLE vio_frontend_node
)

add_executable(tracker_check
  src/tracker_check.cpp
  src/feature_tracker.cpp
)
target_link_libraries(tracker_check ${OpenCV_LIBS})
rosidl_target_interfaces(tracker_check
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(tracker_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tracker_check
  "rclcpp"
  "sensor_msgs"
)

# Install shared library to lib/ so it can be found
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables to lib/${PROJECT_NAME} (libexec)
install(TARGETS tracker_check
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  
  # ament_add_gtest(test_preintegration test/test_preintegration.cpp)
  # target_link_libraries(test_preintegration ${PROJECT_NAME}_lib Eigen3::Eigen)
endif()

ament_package()
