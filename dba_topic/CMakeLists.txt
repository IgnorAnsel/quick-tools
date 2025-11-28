cmake_minimum_required(VERSION 3.8)
project(dba_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-11.8")
set(CUDA_INCLUDE_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include")
set(CUDA_CUDART_LIBRARY "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so")

list(APPEND CMAKE_PREFIX_PATH "${CUDA_TOOLKIT_ROOT_DIR}")
find_package(CUDA QUIET)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV found! Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
add_executable(dba_usb src/dba_usb.cpp src/AndroidCameraClient.cpp)
target_include_directories(dba_usb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dba_usb PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dba_usb
  "rclcpp"
        OpenCV
)

install(TARGETS dba_usb
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
