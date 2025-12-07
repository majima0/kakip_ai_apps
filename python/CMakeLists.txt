cmake_minimum_required(VERSION 3.21)
project(pydrpai CXX)

set(CMAKE_CXX_STANDARD 17)
set(PIP_WHEEL pydrpai-1.0.0-py3-none-any.whl)

set(OpenCVA_INCLUDE_DIRS /usr/include/renesas/opencv4)
set(OpenCVA_LIBS opencv_imgcodecs opencv_imgproc opencv_core opencv_highgui opencv_videoio)

set(TVM_ROOT ../3rdparty/rzv_drp-ai_tvm/tvm)
include_directories(${TVM_ROOT}/include)
include_directories(${TVM_ROOT}/3rdparty/dlpack/include)
include_directories(${TVM_ROOT}/3rdparty/dmlc-core/include)
include_directories(${TVM_ROOT}/3rdparty/compiler-rt)

set(TVM_RUNTIME_LIB /usr/lib/aarch64-linux-gnu/renesas/libtvm_runtime.so)

file(GLOB MY_SOURCE src/*)

add_library(${PROJECT_NAME} SHARED ${MY_SOURCE})
include_directories(
    ../3rdparty/pybind11/include/pybind11
    ../3rdparty/pybind11/include/
    /usr/include/python3.12
    /usr/include/renesas
    ${OpenCVA_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME} PUBLIC /usr/lib/aarch64-linux-gnu/renesas)
target_link_libraries(${PROJECT_NAME} mmngr mmngrbuf)
target_link_libraries(${PROJECT_NAME} python3.12)
target_link_libraries(${PROJECT_NAME} /usr/lib/aarch64-linux-gnu/renesas/libtvm_runtime.so)
target_link_libraries(${PROJECT_NAME} ${OpenCVA_LIBS})

set_target_properties(${PROJECT_NAME} PROPERTIES
        PREFIX ""
        SUFFIX ".so"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE V2H)
