SET(execName Capture)
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
FILE(GLOB HDR_FILES *.h)

add_executable(${execName} ${HDR_FILES} ${SRC_FILES})
target_link_libraries(${execName} ${LINK_LIBS})