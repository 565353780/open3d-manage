cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "-std=c++17 -O0")
project(Open3DManage)

set(Open3D_ROOT ../../open3d-manage/3rd/open3d-devel-linux/)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(D_GLIBCXX_USE_CXX11_ABI 0)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

find_package(Open3D REQUIRED)
if (Open3D_FOUND)
  message(STATUS "Found Open3D ${Open3D_VERSION}")
  link_directories(${Open3D_LIBRARY_DIRS})
  include_directories(${Open3D_INCLUDE_DIRS})
endif()

# openmp
find_package(OpenMP REQUIRED)

# Find the CGAL library
find_package(CGAL REQUIRED)

# Include CGAL's directories and libraries
include(${CGAL_USE_FILE})

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

include_directories(./include/)

include_directories(./3rd/NumCpp/include/)

file(GLOB_RECURSE SRC ./src/*.cpp)
list(FILTER SRC EXCLUDE REGEX "main.cpp")
file(GLOB_RECURSE CURVATURE_SRC ./src/Curvature/*.cpp)

# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
if(WIN32)
    get_target_property(open3d_type Open3D::Open3D TYPE)
    if(open3d_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        add_custom_command(TARGET Draw POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy
                                   ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                   ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()

add_executable(${PROJECT_NAME} ./src/main.cpp ${SRC} ${CURVATURE_SRC})

find_library(TORCH_PYTHON_LIBRARY torch_python PATH "${TORCH_INSTALL_PREFIX}/lib")

target_link_libraries(${PROJECT_NAME} PRIVATE Open3D::Open3D)

target_link_libraries(${PROJECT_NAME} PRIVATE ${TORCH_LIBRARIES} ${TORCH_PYTHON_LIBRARY})
target_link_libraries(${PROJECT_NAME} PRIVATE pybind11::embed)

target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(${PROJECT_NAME} PRIVATE ${CGAL_LIBRARIES})
