cmake_minimum_required(VERSION 3.5)
project(Robot2)
add_compile_options(-std=c++11)

#oi
if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_win32 -std=c++11 -fpermissive")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_win32 -std=c99")
    set(CMAKE_EXE_LINKER_FLAGS "-lwinmm")
endif(WIN32)

#-std=c++11
#set(CMAKE_CXX_FLAGS "-std=gnu++1y")

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_linux -std=c++11 -pthread -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_linux -std=c99 -pthread -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem")
    SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem" )
endif(UNIX)


#nao-sdk tries
#find_package(qibuild REQUIRED)

#set(_qibuild_path "/home/thales/.local/share/cmake")
#list(FIND CMAKE_MODULE_PATH "${_qibuild_path}" _found)
#if(_found STREQUAL "-1")
	#list(INSERT CMAKE_MODULE_PATH 0 "${_qibuild_path}")
	#endif()

# Dependencies:
#list(FIND CMAKE_PREFIX_PATH "/home/thales/TCC/naoqiworktree/naoqi-sdk/naoqi-sdk-2.1.4.13-linux64" _found)
#if(_found STREQUAL "-1")
	#list(APPEND CMAKE_PREFIX_PATH "/home/thales/TCC/naoqiworktree/naoqi-sdk/naoqi-sdk-2.1.4.13-linux64")
	  #endif()

# Store CMAKE_MODULE_PATH and CMAKE_PREFIX_PATH in cache:
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} CACHE INTERNAL ""  FORCE)
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} CACHE INTERNAL ""  FORCE)

#set(QIBUILD_PYTHON_PATH "/home/thales/.local/lib/python2.7/site-packages" CACHE STRING "" FORCE)




set(fuzzylite /home/thales/Documentos/fuzzylite-6.0)
set(naosdk /home/thales/TCC/naoqiworktree/naoqi-sdk/naoqi-sdk-2.1.4.13-linux64/lib /home/thales/TCC/naoqiworktree/naoqi-sdk/naoqi-sdk-2.1.4.13-linux64)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_BINARY_DIR} -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem")

set(SOURCE_FILES
        main.cpp
        src/Simulator.cpp
	src/teste.cpp
	)

find_package(Threads REQUIRED)
find_package(octomap REQUIRED)

find_package(octovis REQUIRED)
#find_package(Boost COMPONENTS system REQUIRED)

include_directories(includes)
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${fuzzylite}/release/bin ${fuzzylite}/fuzzylite)

#include_directories(${naosdk}/lib ${naosdk}/include )
#link_directories(${naosdk}/lib ${naosdk}/include)

link_directories(${fuzzylite}/release/bin ${fuzzylite}/fuzzylite)

link_libraries(${OCTOMAP_LIBRARIES})

add_executable(robot2 ${SOURCE_FILES})
add_library(remote_api src/v_rep/extApi.c src/v_rep/extApiPlatform.c)

if(WIN32)
    target_link_libraries(robot2 remote_api libwinmm.lib ws2_32.lib libhid.lib libsetupapi.lib)
endif(WIN32)
if(UNIX)
	target_link_libraries(robot2 remote_api -lssl -lcrypto -ldl -lboost_iostreams -lboost_system -lboost_filesystem fuzzylite)
endif(UNIX)

#qi_use_lib(robot2 ALCOMMON)

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


