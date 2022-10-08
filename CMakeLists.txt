cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(NAME DICOM)
set(LIBRARY_NAME DICOM)
project(${NAME} VERSION 0.1.0)
set(${NAME}_VERSION_ABI 1)

# C++ 14
set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(Brayns REQUIRED)
find_package(DCMTK REQUIRED)

# Build type and options
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif(NOT CMAKE_BUILD_TYPE)

# Compiler options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Source
set(${NAME}_SOURCES
  DICOMPlugin.cpp
  io/DICOMLoader.cpp
)

set(${NAME}_PUBLIC_HEADERS
  DICOMPlugin.h
  io/DICOMLoader.h
)

# Dependencies
set(${NAME}_LINK_LIBRARIES
  braynsParameters braynsPluginAPI braynsCommon braynsEngine
  ${DCMTK_LIBRARIES})

# Library
include_directories(
    ${BRAYNS_INCLUDE_DIRS}
    ${HDF5_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR})
add_library(
    ${LIBRARY_NAME}
    SHARED ${${NAME}_SOURCES})
target_link_libraries(
    ${LIBRARY_NAME}
    ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
    ${${NAME}_LINK_LIBRARIES})

# ================================================================================
# Install binaries
# ================================================================================
INSTALL(TARGETS ${LIBRARY_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

# Install headers
INSTALL(FILES ${${NAME}_PUBLIC_HEADERS} DESTINATION include/${LIBRARY_NAME})
