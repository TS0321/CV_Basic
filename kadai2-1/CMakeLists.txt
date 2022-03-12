SET(execName kadai2-1)
cmake_minimum_required(VERSION 2.8...3.21)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
PROJECT(${execName})

link_directories(${LIBRARY_OUTPUT_PATH})

if(WIN32)
set(LINK_LIBS
	${YAML_CPP_LIBRARIES}
	${OpenCV_LIBS}
)
endif(WIN32)

FILE(GLOB SRC_FILES *.cpp)
source_group("Source" FILES ${SRC_FILES})
FILE(GLOB HDR_FILES *.h)
source_group("Header" FILES ${HDR_FILES})

FILE(GLOB LIB_SRC_FILES ../CvLib/*.cpp)
source_group("LibSoruce" FILES ${LIB_SRC_FILES})
FILE(GLOB LIB_HDR_FILES ../CvLib/*.h)
source_group("LibHeader" FILES ${LIB_HDR_FILES})


add_executable(${execName} ${HDR_FILES} ${SRC_FILES} ${LIB_SRC_FILES} ${LIB_HDR_FILES})
target_link_libraries(${execName} ${LINK_LIBS})