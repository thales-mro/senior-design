##
# Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved.


cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

add_compile_options(-std=gnu++0x)

# Give a name to the project.
project(movehead)
# This include enable you to use our CMake framework
find_package(qibuild)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTION=255 -D_Linux -pthread -std=c++11 -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_linux -std=c99 -pthread -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem")
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lssl -lcrypto -lboost_iostreams -lboost_system -lboost_filesystem" )

set(fuzzylite /home/thales/Documentos/fuzzylite-6.0-recompiled)

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
include_directories(source/includes)
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${fuzzylite}/fuzzylite)

find_library(FuzzyLiteLibrary fuzzylite HINTS ${fuzzylite}/fuzzylite/release/bin)

message("FuzzyLiteLibrary=${FuzzyLiteLibrary}")

link_directories(${fuzzylite}/fuzzylite/release/bin)

#link_libraries(${OCTOMAP_LIBRARIES})



# Create an executable named movehead,
# with the source file : movehead.cpp
qi_create_bin(movehead movehead.cpp source/src/Simulator.cpp source/src/teste.cpp)

# Tell CMake that movehead depends on ALCOMMON and ALPROXIES
# This will set the libraries to link movehead with,
# the include paths, and so on
qi_use_lib(movehead ALCOMMON ALPROXIES )

add_library(remote_api source/src/v_rep/extApi.c source/src/v_rep/extApiPlatform.c)

target_link_libraries(movehead remote_api -lssl -lcrypto -ldl -lboost_iostreams -lboost_system -lboost_filesystem ${FuzzyLiteLibrary})


set_target_properties( remote_api
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties( movehead
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

