cmake_minimum_required(VERSION 3.15)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(OpenCVA_INCLUDE_DIRS /usr/include/renesas/opencv4)
set(OpenCVA_LIBS opencv_imgcodecs opencv_imgproc opencv_core opencv_highgui opencv_videoio)

set(CURRENT_DIR "" CACHE STRING "project dir")
if(NOT CURRENT_DIR)
	message(FATAL_ERROR "not set CURRENT_DIR")
endif()

set(TVM_ROOT ${CURRENT_DIR}/../3rdparty/rzv_drp-ai_tvm/tvm)
include_directories(${TVM_ROOT}/include)
include_directories(${TVM_ROOT}/3rdparty/dlpack/include)
include_directories(${TVM_ROOT}/3rdparty/dmlc-core/include)
include_directories(${TVM_ROOT}/3rdparty/compiler-rt)

set(TVM_RUNTIME_LIB /usr/lib/aarch64-linux-gnu/renesas/libtvm_runtime.so)

file(GLOB MY_SOURCE src/*)

pybind11_add_module(_core MODULE ${MY_SOURCE})
install(TARGETS _core DESTINATION ${SKBUILD_PROJECT_NAME})

target_compile_definitions(_core PRIVATE V2H)

include_directories(
    ${CURRENT_DIR}/../3rdparty/pybind11/include/pybind11
    ${CURRENT_DIR}/../3rdparty/pybind11/include/
    /usr/include/python3.12
    /usr/include/renesas
    ${OpenCVA_INCLUDE_DIRS}
)

target_link_directories(_core PUBLIC /usr/lib/aarch64-linux-gnu/renesas)
target_link_libraries(_core PUBLIC mmngr mmngrbuf)
target_link_libraries(_core PUBLIC python3.12)
target_link_libraries(_core PUBLIC /usr/lib/aarch64-linux-gnu/renesas/libtvm_runtime.so)
target_link_libraries(_core PUBLIC ${OpenCVA_LIBS})

