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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Feature.msg"
  "msg/FeatureMeasurement.msg"
  DEPENDENCIES std_msgs
)

add_executable(estimator 
  src/estimator.cpp
  src/feature_tracker.cpp
)
target_include_directories(estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  estimator
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "geometry_msgs"
  "nav_msgs"
)
target_link_libraries(estimator ${OpenCV_LIBS})
rosidl_target_interfaces(estimator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

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



# Feature Tracker Component Library
add_library(feature_tracker_component SHARED
  src/feature_tracker_node.cpp 
  src/feature_tracker.cpp
)
target_include_directories(feature_tracker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(feature_tracker_component ${OpenCV_LIBS})
ament_target_dependencies(feature_tracker_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)
rosidl_target_interfaces(feature_tracker_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Feature Tracker Node Executable
add_executable(feature_tracker_node src/feature_tracker_main.cpp)
target_link_libraries(feature_tracker_node feature_tracker_component)
ament_target_dependencies(feature_tracker_node "rclcpp")

install(TARGETS estimator tracker_check feature_tracker_node feature_tracker_component
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
