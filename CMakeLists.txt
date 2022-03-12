cmake_minimum_required(VERSION 2.8...3.21)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(the_target "CV_Basic")
project(${the_target})

include(cmake/cmakeinc.cmake)

ADD_SUBDIRECTORY(Capture)
ADD_SUBDIRECTORY(kadai1-1)

