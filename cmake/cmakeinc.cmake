# set the project name
project(${the_target})

# Find modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_MODULE_PATH})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

##### Windows環境での依存関係の読み込み #####
if(WIN32)
INCLUDE_DIRECTORIES(
    "${CMAKE_CURRENT_LIST_DIR}/../ThirdParty/include"
)
LINK_DIRECTORIES(
    "${CMAKE_CURRENT_LIST_DIR}/../ThirdParty/lib64"
)
endif(WIN32)

## OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif(OpenCV_FOUND)

## Ceres & glog
if(WIN32)
## Eigen
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/../ThirdParty/include)
endif(WIN32)

## yaml-cpp
find_package(yaml-cpp REQUIRED)
if(YAML_CPP_FOUND)
    include_directories(${YAML_CPP_INCLUDE_DIRS})
    link_directories(${YAML_CPP_LIBRARIES})
endif(YAML_CPP_FOUND)

## realsense
find_package(realsense2 REQUIRED)
if(REALSENSE2_FONUD)
    include_directories(${REALSENSE2_INCLUDE_DIRS})
    link_directories(${REALSENSE2_LIBRARIES})
endif(REALSENSE2_FONUD)
