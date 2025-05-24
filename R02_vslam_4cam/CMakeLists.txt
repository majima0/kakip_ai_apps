cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_FIND_ROOT_PATH: ${CMAKE_FIND_ROOT_PATH}")
message(STATUS "CMAKE_SYSTEM_PREFIX_PATH: ${CMAKE_SYSTEM_PREFIX_PATH}")
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_PACKAGE: ${CMAKE_FIND_ROOT_PATH_MODE_PACKAGE}")
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_LIBRARY: ${CMAKE_FIND_ROOT_PATH_MODE_LIBRARY}")
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_INCLUDE: ${CMAKE_FIND_ROOT_PATH_MODE_INCLUDE}")
message(STATUS "CMAKE_FIND_ROOT_PATH_MODE_PROGRAM: ${CMAKE_FIND_ROOT_PATH_MODE_PROGRAM}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# To suppress the warning of cereal
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-class-memaccess")

# Check C++11 or C++0x support
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
set(CMAKE_CXX_STANDARD 17)

set(ENABLE_DUMP OFF CACHE BOOL "Enable dump in/out data of GaussianBlur, FAST, computeOrbDescriptors, ComputePyramid and YOLO")
if(ENABLE_DUMP)
    add_definitions(-DENABLE_DUMP)
    add_definitions(-DENABLE_DEBUG_OUTPUT)
    message(STATUS "Using ENABLE_DUMP.")
    message(STATUS "Using ENABLE_DEBUG_OUTPUT.")
endif()

set(ENABLE_MEASURE_TIME OFF CACHE BOOL "Enable measure execution time of GaussianBlur, FAST, computeOrbDescriptors and ComputePyramid")
if(ENABLE_MEASURE_TIME)
    add_definitions(-DENABLE_MEASURE_TIME)
    message(STATUS "Using ENABLE_MEASURE_TIME.")
endif()

set(ENABLE_DRP OFF CACHE BOOL "Use CPU-DRP cooperation")
if(ENABLE_DRP)
    add_definitions(-DENABLE_DRP)
    message(STATUS "Using ENABLE_DRP.")
endif()

set(ENABLE_DRP_AI OFF CACHE BOOL "Use CPU-DRP-AI cooperation")
if(ENABLE_DRP_AI)
    add_definitions(-DENABLE_DRP_AI)
    message(STATUS "Using ENABLE_DRP_AI.")
endif()

set(ENABLE_REALSENSE2 OFF CACHE BOOL "Use RealSense2 camera")
if(ENABLE_REALSENSE2)
    add_definitions(-DENABLE_REALSENSE2)
    message(STATUS "Using ENABLE_REALSENSE2.")
endif()

set(ENABLE_GOOGLE_PERF OFF CACHE BOOL "Enable profiling of google-perftools")
if(ENABLE_GOOGLE_PERF)
  add_definitions(-DENABLE_GOOGLE_PERF)
  message(STATUS "Using ENABLE_GOOGLE_PERF.")
endif()

set(ENABLE_YOCTO ON CACHE BOOL "Enable build with yocto")
if(ENABLE_YOCTO)
  add_definitions(-DENABLE_YOCTO)
  message(STATUS "Using ENABLE_YOCTO.")
endif()

set(ENABLE_SLAMFAST OFF CACHE BOOL "Use SLAM FAST")
if(ENABLE_SLAMFAST)
  add_definitions(-DENABLE_SLAMFAST)
  message(STATUS "Using ENABLE_SLAMFAST.")
endif()

set(ENABLE_TVM OFF CACHE BOOL "Use DRP-AI TVM Runtime")
if(ENABLE_TVM)
  add_definitions(-DENABLE_TVM)
  message(STATUS "Using ENABLE_TVM.")
  message(WARNING "This is an experimental setup.")
  message(WARNING "This operation is not guaranteed.")
endif()

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(OpenGL REQUIRED opengl)
#pkg_check_modules(PC_PKG REQUIRED glx)
#find_package(OpenGL REQUIRED)

#find_package(VTK REQUIRED)
#set(VTK_MODULE_ENABLE_VTK_GuiSupportQt YES)
#set(VTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2 YES)
#set(VTK_MODULE_ENABLE_VTK_RenderingOpenGL2 YES)
#find_package(flann NO_MODULE REQUIRED)
#message("LIBLZ4_LIBRARIES:" ${LIBLZ4_LIBRARIES})

find_package(OpenCV 4.1.0 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
# find_package(Pangolin REQUIRED)
#find_package(PCL 1.9.1 REQUIRED)
find_package(PCL 1.12.1 REQUIRED COMPONENTS common filters)

message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "${CMAKE_LIBRARY_PATH}")
#find_package(sioclient REQUIRED)
# Change SIOCLIENT_LIBRARY for BitBake
#set(SIOCLIENT_LIBRARY "sioclient")

find_package(Protobuf REQUIRED)
if(ENABLE_REALSENSE2)
  find_package(realsense2 REQUIRED)
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/g2oAddition/include
${PROJECT_SOURCE_DIR}/CAPEAddition/include
${PROJECT_SOURCE_DIR}/socket_modules
${PROJECT_SOURCE_DIR}/image_load_modules
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
# ${Pangolin_INCLUDE_DIRS}
#${SIOCLIENT_INCLUDE_DIR}
${PROTOBUF_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/Thirdparty/sioclient/src
)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/sioclient)

if(NOT ENABLE_YOCTO AND (ENABLE_DRP OR ENABLE_DRP_AI))
  include_directories(${PROJECT_SOURCE_DIR}/v2x)
endif()

if(ENABLE_DRP)
  include_directories(${PROJECT_SOURCE_DIR}/drp_modules)
  include_directories(${PROJECT_SOURCE_DIR}/image_proc_modules)
  include_directories(${PROJECT_SOURCE_DIR}/opencva_modules)
endif()

if(ENABLE_DRP_AI)
  include_directories(${PROJECT_SOURCE_DIR}/drp_ai_modules)
  include_directories(${PROJECT_SOURCE_DIR}/drp_ai_modules/include)
endif()

if(ENABLE_REALSENSE2)
  include_directories(${PROJECT_NAME} ${realsense_INCLUDE_DIR})
endif()

add_subdirectory(socket_modules)
add_subdirectory(image_load_modules)

if(ENABLE_DRP)
  add_subdirectory(drp_modules)
  add_subdirectory(image_proc_modules)
  add_subdirectory(opencva_modules)
endif()

if(ENABLE_DRP_AI)
  add_subdirectory(drp_ai_modules)
endif()

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/YoloDetector.cc
src/MapPlane.cc
src/PlaneMatcher.cc
src/ConfigFile.cc
g2oAddition/src/EdgePlane.cc
g2oAddition/src/VertexPlane.cc
CAPEAddition/src/CAPE.cpp
CAPEAddition/src/PlaneSeg.cpp
CAPEAddition/src/CylinderSeg.cpp
CAPEAddition/src/Histogram.cpp
src/CameraInfo.cc
)

#message("OpenCV: " "${OpenCV_LIBRARIES}")

target_link_libraries(${PROJECT_NAME}
PUBLIC
socket_modules
image_load_modules
#${OpenCV_LIBS}
#${EIGEN3_LIBS}
${OpenCV_LIBRARIES}
${EIGEN3_LIBRARIES}
# ${Pangolin_LIBRARIES}
#${SIOCLIENT_LIBRARY}
sioclient
${PROTOBUF_LIBRARY}
${PROTOBUF_LIBRARIES}
DBoW2
g2o
${PCL_LIBRARIES}
rt # Please refer to 20230508_V2H_YOLOv2_demo/rzv2h_yolov2_demo_src_20230508/src/CMakeLists.txt
)

if(ENABLE_DRP)
  target_link_libraries(${PROJECT_NAME} PUBLIC drp_modules image_proc_modules)
  target_link_libraries(${PROJECT_NAME} PUBLIC opencva_modules)
endif()

if(ENABLE_DRP_AI)
  target_link_libraries(${PROJECT_NAME} PUBLIC drp_ai_modules)
endif()

if(ENABLE_REALSENSE2)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${realsense2_LIBRARY})
endif()

if(ENABLE_GOOGLE_PERF)
  target_link_libraries(${PROJECT_NAME} PUBLIC profiler)
endif()

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_definitions(-DEIGEN_DONT_PARALLELIZE)
#add_definitions(-DENABLE_DEBUG_OUTPUT)

target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--no-as-needed")

# Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

#if(ENABLE_REALSENSE2)
#  add_executable(rgbd_realsense
#  Examples/RGB-D/rgbd_realsense.cc)
#  target_link_libraries(rgbd_realsense ${PROJECT_NAME})
#endif()

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_euroc
#Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})

#add_executable(mono_usbcam
#Examples/Monocular/mono_usbcam.cc)
#target_link_libraries(mono_usbcam ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Multicam)

add_executable(multicam
Examples/Multicam/multicam.cc)
target_link_libraries(multicam ${PROJECT_NAME})
target_link_options(multicam PRIVATE "-Wl,--unresolved-symbols=ignore-in-shared-libs")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)

add_executable(calibrate_single
Demo/calibrate.cc
Examples/Calibration/calibrate_single.cc)
target_include_directories(calibrate_single PRIVATE ${PROJECT_SOURCE_DIR}/Demo)
target_link_libraries(calibrate_single ${PROJECT_NAME})
target_link_options(calibrate_single PRIVATE "-Wl,--unresolved-symbols=ignore-in-shared-libs")

add_executable(calibrate_multi
Demo/calibrate.cc
Examples/Calibration/calibrate_multi.cc)
target_include_directories(calibrate_multi PRIVATE ${PROJECT_SOURCE_DIR}/Demo)
target_link_libraries(calibrate_multi ${PROJECT_NAME})
target_link_options(calibrate_multi PRIVATE "-Wl,--unresolved-symbols=ignore-in-shared-libs")

#set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${YOLO_PLANAR_SLAM_VERSION})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Demo)
add_executable(kakip_vslam_demo
Demo/kakip_vslam_demo.cc
Demo/calibrate.cc)
target_include_directories(kakip_vslam_demo PRIVATE ${PROJECT_SOURCE_DIR}/Thirdparty/clx)
target_link_libraries(kakip_vslam_demo ${PROJECT_NAME})
target_link_options(kakip_vslam_demo PRIVATE "-Wl,--unresolved-symbols=ignore-in-shared-libs")

add_executable(kakip_vslam_demo_with_rasp
Demo/kakip_vslam_demo_with_rasp.cc
Demo/calibrate.cc)
target_include_directories(kakip_vslam_demo_with_rasp PRIVATE ${PROJECT_SOURCE_DIR}/Thirdparty/clx)
target_link_libraries(kakip_vslam_demo_with_rasp ${PROJECT_NAME})
target_link_options(kakip_vslam_demo_with_rasp PRIVATE "-Wl,--unresolved-symbols=ignore-in-shared-libs")