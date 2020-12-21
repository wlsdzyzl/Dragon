cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_BINARY_DIR "./app")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})


include_directories(3rdparty/Eigen)
include_directories(3rdparty/tinyply/source)
include_directories(3rdparty/tinyobjloader)
include_directories(3rdparty/Sophus)
include_directories(3rdparty/imgui)
include_directories(3rdparty/nanoflann/include)
include_directories(src)

#message("tinyobjloader")



file(GLOB_RECURSE Geometry_SRC
"src/Geometry/*.cpp"
"src/Geometry/*.h"
"src/Geometry/*.hpp"
)

file(GLOB_RECURSE Visualization_SRC
"src/Visualization/Shaders/*.h"
"src/Visualization/*.cpp"
"src/Visualization/*.h"
)
file(GLOB Tool_SRC
"src/Tool/*.cpp"
"src/Tool/*.h"
)
file(GLOB Objloader_SRC
"3rdparty/tinyobjloader/*.c"
"3rdparty/tinyobjloader/*.cpp"
"3rdparty/tinyobjloader/*.cc"
"3rdparty/tinyobjloader/*.h"
)
file(GLOB TinyPly_SRC
"3rdparty/tinyply/source/tinyply.h"
"3rdparty/tinyply/source/tinyply.cpp"
)
file(GLOB IO_SRC
"src/IO/*.cpp"
"src/IO/*.h"
)
add_subdirectory(3rdparty/glfw)
add_subdirectory(3rdparty/imgui)

add_library(tinyply SHARED ${TinyPly_SRC})
add_library(objloader SHARED ${Objloader_SRC})



add_library(dragon SHARED ${Geometry_SRC}  
    ${Visualization_SRC}  ${IO_SRC} ${Tool_SRC})



set(CMAKE_CXX_FLAGS " -std=c++11 ")
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -O0 -g -Wall -fopenmp -Wno-maybe-uninitialized") 
# if you want a double-precision
# set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -DUSING_FLOAT64") 
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(ReadMesh "app/ReadPLYMesh.cpp")
# for test
# add_executable(GenerateMinimalSurface "app/GenerateMinimalSurface.cpp")
add_executable(MeanCurvature "app/MeanCurvature.cpp")
# add_executable(GaussCurvature "app/GaussCurvature.cpp")
add_executable(DragonView3D "app/DragonView.cpp")
add_executable(DragonView2D "app/DragonView2D.cpp")
set(THIRD_LIB 
    tinyply 
    objloader
    imgui
    glfw
    ${OpenCV_LIBS} 
    ${GLUT_LIBRARIES}  
    ${GLEW_LIBRARIES} 
    ${OPENGL_LIBRARIES} 
)

target_link_libraries(ReadMesh
    dragon
    ${THIRD_LIB}
)
target_link_libraries(MeanCurvature
    dragon
    ${THIRD_LIB}
)
target_link_libraries(DragonView3D
    dragon
    ${THIRD_LIB}
)
target_link_libraries(DragonView2D
    dragon
    ${THIRD_LIB}
)