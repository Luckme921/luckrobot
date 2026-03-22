# Install script for directory: /home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/orbbec_camera_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE PROGRAM FILES
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py"
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/service_benchmark_node.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/frame_latency_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/frame_latency_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/frame_latency_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/frame_latency_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/frame_latency_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/frame_latency_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/frame_latency_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/frame_latency_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/start_benchmark_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/start_benchmark_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/start_benchmark_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/start_benchmark_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/start_benchmark_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/start_benchmark_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/start_benchmark_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/start_benchmark_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/multi_save_rgbir_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/multi_save_rgbir_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/multi_save_rgbir_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/multi_save_rgbir_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/multi_save_rgbir_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/multi_save_rgbir_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/multi_save_rgbir_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/multi_save_rgbir_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/liborbbec_camera.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libframe_latency.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libframe_latency.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libframe_latency.so"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/libframe_latency.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libframe_latency.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libframe_latency.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libframe_latency.so"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libframe_latency.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libstart_benchmark.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libstart_benchmark.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libstart_benchmark.so"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/libstart_benchmark.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libstart_benchmark.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libstart_benchmark.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libstart_benchmark.so"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libstart_benchmark.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmulti_save_rgbir.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmulti_save_rgbir.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmulti_save_rgbir.so"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/libmulti_save_rgbir.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmulti_save_rgbir.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmulti_save_rgbir.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmulti_save_rgbir.so"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmulti_save_rgbir.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/" TYPE DIRECTORY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/" TYPE DIRECTORY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/" TYPE DIRECTORY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/" TYPE DIRECTORY FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/" FILES_MATCHING REGEX "/[^/]*\\.so[^/]*$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/extensions" TYPE FILE FILES
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/extensions/depthengine/libdepthengine.so"
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/extensions/depthengine/libdepthengine.so.2.0"
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/extensions/filters/libFilterProcessor.so"
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/extensions/filters/libob_priv_filter.so"
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/extensions/firmwareupdater/libfirmwareupdater.so"
    "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/extensions/frameprocessor/libob_frame_processor.so"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/udev" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/list_devices_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/list_depth_work_mode_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/list_camera_profile_mode_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/topic_statistics_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/topic_statistics_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/topic_statistics_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/topic_statistics_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/topic_statistics_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/topic_statistics_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/topic_statistics_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/topic_statistics_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/service_benchmark_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/service_benchmark_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/service_benchmark_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/service_benchmark_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/service_benchmark_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/service_benchmark_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/service_benchmark_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/service_benchmark_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/ob_benchmark_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/ob_benchmark_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/ob_benchmark_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ob_benchmark_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/ob_benchmark_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/ob_benchmark_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/ob_benchmark_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/ob_benchmark_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/435le_example_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/435le_example_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/435le_example_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/435le_example_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/435le_example_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/435le_example_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/435le_example_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/435le_example_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/set_device_ip" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/set_device_ip")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/set_device_ip"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/set_device_ip")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/set_device_ip" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/set_device_ip")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/set_device_ip"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/set_device_ip")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/image_sync_example_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/image_sync_example_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/image_sync_example_node"
         RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/image_sync_example_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/image_sync_example_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/image_sync_example_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/image_sync_example_node"
         OLD_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64:/usr/local/cuda-12.6/lib64:/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/home/nvidia/luckrobot/orbbecsdk_ws/install/orbbec_camera_msgs/lib:/usr/local/lib:"
         NEW_RPATH ":/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/image_sync_example_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/environment" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/environment" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/environment" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_index/share/ament_index/resource_index/packages/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rclcpp_components" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_index/share/ament_index/resource_index/rclcpp_components/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/cmake" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/cmake" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/cmake" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/cmake" TYPE FILE FILES
    "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_core/orbbec_cameraConfig.cmake"
    "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/ament_cmake_core/orbbec_cameraConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/nvidia/luckrobot/orbbecsdk_ws/src/OrbbecSDK_ROS2/orbbec_camera/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nvidia/luckrobot/orbbecsdk_ws/build/orbbec_camera/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
