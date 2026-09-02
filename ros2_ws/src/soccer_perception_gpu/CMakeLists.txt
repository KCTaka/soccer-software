cmake_minimum_required(VERSION 3.16)
project(soccer_perception_gpu CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(soccer_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# --- CUDA + TensorRT -------------------------------------------------------
# Located by hand rather than via find_package(CUDAToolkit): we only need the
# runtime and the inference library, and enabling the CUDA language would force
# every consumer to have nvcc. NvInfer.h comes from libnvinfer-headers-dev,
# which must be pinned to the same major.minor as the installed libnvinfer
# runtime, or engines fail to deserialise at run time.
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu
  /usr/local/tensorrt/include)
find_library(TENSORRT_LIBRARY nvinfer
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/tensorrt/lib)
find_path(CUDA_INCLUDE_DIR cuda_runtime.h PATHS /usr/local/cuda/include /usr/include)
find_library(CUDART_LIBRARY cudart
  PATHS /usr/local/cuda/lib64 /usr/local/cuda/targets/aarch64-linux/lib /usr/lib/aarch64-linux-gnu)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT CUDA_INCLUDE_DIR OR NOT CUDART_LIBRARY)
  message(FATAL_ERROR
    "soccer_perception_gpu needs CUDA + TensorRT headers and libraries.\n"
    "  NvInfer.h        : ${TENSORRT_INCLUDE_DIR}\n"
    "  libnvinfer       : ${TENSORRT_LIBRARY}\n"
    "  cuda_runtime.h   : ${CUDA_INCLUDE_DIR}\n"
    "  libcudart        : ${CUDART_LIBRARY}\n"
    "This package only builds inside the Jetson camera image "
    "(deploy/docker/Dockerfile.jetson, stage zed-driver-image). "
    "Exclude it elsewhere with --packages-skip soccer_perception_gpu.")
endif()
message(STATUS "TensorRT: ${TENSORRT_LIBRARY} (headers ${TENSORRT_INCLUDE_DIR})")

set(deps rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs soccer_msgs cv_bridge)

add_library(trt_engine SHARED src/trt_engine.cpp)
target_include_directories(trt_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIR} ${CUDA_INCLUDE_DIR})
target_link_libraries(trt_engine PUBLIC ${TENSORRT_LIBRARY} ${CUDART_LIBRARY})

add_library(soccer_perception_gpu_components SHARED
  src/detector_component.cpp
  src/fieldline_component.cpp)
target_include_directories(soccer_perception_gpu_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(soccer_perception_gpu_components PUBLIC trt_engine ${OpenCV_LIBS})
ament_target_dependencies(soccer_perception_gpu_components PUBLIC ${deps})

rclcpp_components_register_node(soccer_perception_gpu_components
  PLUGIN "soccer_perception_gpu::DetectorComponent"
  EXECUTABLE detector_gpu_node)
rclcpp_components_register_node(soccer_perception_gpu_components
  PLUGIN "soccer_perception_gpu::FieldlineComponent"
  EXECUTABLE fieldline_gpu_node)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS trt_engine soccer_perception_gpu_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_model test/test_camera_model.cpp)
  if(TARGET test_camera_model)
    target_include_directories(test_camera_model PRIVATE include)
    ament_target_dependencies(test_camera_model sensor_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(trt_engine soccer_perception_gpu_components)
ament_export_dependencies(${deps})
ament_package()
