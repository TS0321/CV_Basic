if(YAML_CPP_LIBRARIES AND YAML_CPP_INCLUDE_DIRS)
# in cache already
    set(YAML_CPP_FOUND TRUE)
else (YAML_CPP_LIBRARIES AND YAML_CPP_INCLUDE_DIRS)
    if(WIN32)
        find_path(YAML_CPP_INCLUDE_DIR
            NAMES 
                yaml-cpp/yaml.h
            PATHS 
                ${CMAKE_CURRENT_LIST_DIR}/../ThirdParty/include
            )
        find_library(YAML_CPP_LIBRARY
            NAMES 
                yaml-cpp
            PATHS 
                ${CMAKE_CURRENT_LIST_DIR}/../ThirdParty/lib64
            )
    endif(WIN32)

    set(YAML_CPP_INCLUDE_DIRS
            ${YAML_CPP_INCLUDE_DIR}
        )
    set(YAML_CPP_LIBRARIES
            ${YAML_CPP_LIBRARY}
        )
    
    if(YAML_CPP_INCLUDE_DIRS AND YAML_CPP_LIBRARIES)
        set(YAML_CPP_FOUND TRUE)
    endif(YAML_CPP_INCLUDE_DIRS AND YAML_CPP_LIBRARIES)

    if(YAML_CPP_FOUND)
        message(STATUS "Found yaml-cpp.lib:")
        message(STATUS "- Includes: ${YAML_CPP_INCLUDE_DIRS}")
        message(STATUS "- Libraries: ${YAML_CPP_LIBRARIES}")
    else(YAML_CPP_FOUND)
        message(FATAL_ERROR "Could not find yaml-cpp.lib")
    endif(YAML_CPP_FOUND)

    mark_as_advanced(YAML_CPP_INCLUDE_DIRS YAML_CPP_LIBRARIES)

    endif(YAML_CPP_LIBRARIES AND YAML_CPP_INCLUDE_DIRS)