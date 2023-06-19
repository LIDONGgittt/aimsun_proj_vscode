cmake_minimum_required(VERSION 3.14)
project(AIMSUN_I66 VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug) # or Release

include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB SOURCES "src/*.cxx" "src/*.cpp")
file(GLOB HEADERS include/*.h)
file(GLOB AIMSUN_LIBS aimsun_lib/*.lib)

message("#### All cxx/cpp files are: " ${SOURCES})
message("#### All Aimsun lib files are: " ${AIMSUN_LIBS})
# foreach(file ${SOURCES})
#     get_filename_component(exename ${file} NAME_WE)
#     add_executable(${exename} ${file})
# endforeach()
string(TOUPPER ${CMAKE_BUILD_TYPE} UPPER_CASE_BUILD_TYPE)

message("#### Current build type is: " ${CMAKE_BUILD_TYPE})
message("#### UPPER_CASE_BUILD_TYPE is: " ${UPPER_CASE_BUILD_TYPE})

set(API_LIB_SUFFIX "")
if (${UPPER_CASE_BUILD_TYPE} STREQUAL "DEBUG")
    message("#### Target library will have a suffix of _D.dll" )
    set(API_LIB_SUFFIX "_D.dll")
elseif (${UPPER_CASE_BUILD_TYPE} STREQUAL "RELEASE")
    message("#### Target library will have a suffix of _R.dll")
    set(API_LIB_SUFFIX "_R.dll")
else()
    message("#### ERROR ### Cannot detect an available build type!!!")
endif()

foreach(file ${SOURCES})
    get_filename_component(sourcename ${file} NAME_WE)
    add_library(${sourcename} SHARED ${file})
    target_link_libraries(${sourcename} ${AIMSUN_LIBS})
    set_target_properties(${sourcename} 
        PROPERTIES 
        PREFIX "" 
        SUFFIX ${API_LIB_SUFFIX}
        ARCHIVE_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/lib"
        )
endforeach()

add_subdirectory(test)