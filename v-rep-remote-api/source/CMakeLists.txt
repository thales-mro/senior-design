cmake_minimum_required(VERSION 3.5)
project(Robot2)

#oi
if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_win32 -std=c++11 -fpermissive")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_win32 -std=c99")
    set(CMAKE_EXE_LINKER_FLAGS "-lwinmm")
endif(WIN32)

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_linux -std=c++11 -pthread -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_linux -std=c99 -pthread -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem")
    SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem" )
endif(UNIX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_BINARY_DIR} -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem")

set(SOURCE_FILES
        main.cpp
        src/Simulator.cpp
        )

find_package(Threads REQUIRED)
find_package(octomap REQUIRED)

find_package(octovis REQUIRED)
#find_package(Boost COMPONENTS system REQUIRED)

include_directories(includes)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

add_executable(robot2 ${SOURCE_FILES})
add_library(remote_api src/v_rep/extApi.c src/v_rep/extApiPlatform.c)

if(WIN32)
    target_link_libraries(robot2 remote_api libwinmm.lib ws2_32.lib libhid.lib libsetupapi.lib)
endif(WIN32)
if(UNIX)
    target_link_libraries(robot2 remote_api -lssl -lcrypto -ldl -lboost_iostreams -lboost_system -lboost_filesystem)
endif(UNIX)

set_target_properties( remote_api
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties( robot2
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)


