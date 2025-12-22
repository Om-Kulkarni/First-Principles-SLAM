cmake_minimum_required(VERSION 3.8)
project(vio_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

# 1. Find Eigen (Matrix Math)
find_package(Eigen3 REQUIRED)
add_definitions(-DEIGEN_NO_DEBUG) # Makes Eigen fast

# 2. Find Ceres (Optimization Backend)
find_package(Ceres REQUIRED)

# 3. Find OpenCV (If you need to draw debug images)
find_package(OpenCV REQUIRED)

find_package(vio_frontend REQUIRED)

add_executable(estimator_node src/estimator_node.cpp)
target_include_directories(estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(estimator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  estimator_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "sensor_msgs"
  "cv_bridge"
  "std_msgs"
  "vio_frontend"
)

target_include_directories(estimator_node PUBLIC
  ${Eigen3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${vio_frontend_INCLUDE_DIRS}
)

target_link_libraries(estimator_node
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${vio_frontend_LIBRARIES}
)

install(TARGETS estimator_node
  DESTINATION lib/${PROJECT_NAME})

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
