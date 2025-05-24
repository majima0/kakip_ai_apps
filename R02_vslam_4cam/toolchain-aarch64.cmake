set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_SYSROOT /opt/sysroots/arm64-ubuntu)
set(CMAKE_FIND_ROOT_PATH /opt/sysroots/arm64-ubuntu)

set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc-11)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++-11)

set(CMAKE_C_FLAGS "--sysroot=${CMAKE_SYSROOT} ${CMAKE_C_FLAGS} -mtune=cortex-a55 -fstack-protector-strong -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security -Wl,-rpath-link=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
set(CMAKE_CXX_FLAGS "--sysroot=${CMAKE_SYSROOT} ${CMAKE_CXX_FLAGS} -mtune=cortex-a55 -fstack-protector-strong -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security -Wl,-rpath-link=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
#set(CMAKE_EXE_LINKER_FLAGS "-v --sysroot=${CMAKE_SYSROOT} ${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
#set(CMAKE_SHARED_LINKER_FLAGS "--sysroot=${CMAKE_SYSROOT} ${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
#set(LD_FLAGS "-rpath-link=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu -rpath=${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
#set(CMAKE_SYSROOT_COMPILE "${CMAKE_FIND_ROOT_PATH}")
#set(CMAKE_SYSROOT_LINK "${CMAKE_FIND_ROOT_PATH}")
#set(CMAKE_INCLUDE_PATH ${CMAKE_SYSROOT}/usr/include)

#set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake;${CMAKE_SYSROOT}/usr/lib/cmake;${CMAKE_SYSROOT}/usr/share/cmake")
#set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/usr/share/cmake-3.22/Modules")

set(CMAKE_CROSSCOMPILING TRUE)

#set(CMAKE_PREFIX_PATH ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake ${CMAKE_SYSROOT}/usr/lib/cmake ${CMAKE_SYSROOT}/usr/share/cmake)
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_SYSROOT}/usr/share/cmake-3.28/Modules/)
#set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/cmake")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}
  ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/cmake
  ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake
)
#  ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/cmake/vtk-9.1
#  ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/cmake/Qt5
#)
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
 "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/renesas"
 "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu"
)
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH}
 "${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu"
 "${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/openmpi"
)

set(PCL_DIR ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake/pcl)

#set(Qt5_DIR "${CMAKE_SYSROOT}/lib/aarch64-linux-gnu/cmake/Qt5")

#set(LIBUSB_1_INCLUDE_DIRS ${CMAKE_SYSROOT}/usr/include/libusb-1.0)
#set(libusb_LIBRARIES ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libusb-1.0.so)
#set(GLEW_LIBRARY ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libGLEW.so)
#set(OPNEGL_opengl_LIBRARY ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libGL.so)
#set(OPENGL_glx_LIBRARY ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libGLX.so)
#set(OpenGL_LIBRARY ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libOpenGL.so)
#set(Python3_INCLUDE_DIR "${CMAKE_FIND_ROOT_PATH}/usr/include/python3.12")
#set(Python3_LIBRARY "${CMAKE_FIND_ROOT_PATH}/usr/lib/aarch64-linux-gnu/libpython3.12.so")
set(Python3_EXECUTABLE "/home/yokoyama/.pyenv/versions/3.12.1/bin/python3.12") 

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi")
#set(MPI_C_COMPILER "${CMAKE_C_COMPILER} --sysroot=${CMAKE_SYSROOT}")
#set(MPI_C_LINK_FLAGS "--sysroot=${CMAKE_SYSROOT} -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi/lib -lmpi")
#set(MPI_C_COMPILE_OPTIONS "--sysroot=${CMAKE_SYSROOT} -I{CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/openmpi")
#set(MPI_C_LINK_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --sysroot=${CMAKE_SYSROOT} -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi/lib -lmpi")
#set(MPI_CXX_COMPILER "${CMAKE_CXX_COMPILER}")
set(MPI_DETERMINE_LIBRARY_VERSION 1)
#set(MPIEXEC_EXECUTABLE "/usr/bin/mpiexec")
set(MPI_C_WORKS TRUE)
set(MPI_C_COMPILER_WORKS TRUE)
set(MPI_C_SKIP_RUN TRUE)
#message("===============" ${MPI_C_COMPILER})
set(MPI_C_INCLUDE_DIRS "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi/include")
set(MPI_C_COMPILER "${CMAKE_C_COMPILER}")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi/include -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi/lib -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
#set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} --sysroot=${CMAKE_SYSROOT}")
#set(MPI_C_COMPILER "-I${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/openmpi -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi/lib -lmpi")
#set(MPI_C_COMPILER "-lmpi")
#set(MPI_C_COMPILER "/usr/bin/mpicc --sysroot=${CMAKE_SYSROOT}")
#set(MPI_C_COMPILER "${CMAKE_SYSROOT}/usr/bin/mpicc")
#set(MPI_CXX_COMPILER "${CMAKE_SYSROOT}/usr/bin/mpicxx")
#set(MPI_C_COMPILER "/usr/in/mpicc --sysroot=${CMAKE_SYSROOT} -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openmpi")
#set(MPI_CXX_COMPILER "/usr/bin/mpicxx")
#set(MPI_DIR "${CMAKE_FIND_ROOT_PATH}/usr/lib/aarch64-linux-gnu/openmpi")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_FIND_ROOT_PATH}/usr/lib/aarch64-linux-gnu/openmpi)
#set(MPI_mpi_LIBRARY "${CMAKE_FIND_ROOT_PATH}/usr/lib/aarch64-linux-gnu/openmpi/lib/libmpi.so")
#set(MPI_C_LIBRARIES "${CMAKE_FIND_ROOT_PATH}/usr/lib/aarch64-linux-gnu/openmpi/lib/libmpi.so")
#set(MPI_EXTRA_LIBRARIES "${CMAKE_FIND_ROOT_PATH}/usr/lib/aarch64-linux-gnu/openmpi/lib/libmpi_cxx.so")
#set(MPI_C_FOUND TRUE)
set(Protobuf_PROTOC_EXECUTABLE ${PROJECT_SOURCE_DIR}/protoc-target.sh CACHE FILEPATH "Protobuf compiler")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

list(APPEND CMAKE_FIND_ROOT_PATH_MODE_PACKAGE_BOTH Python3)