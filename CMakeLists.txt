cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
set(NAME DICOM)
project(${NAME} VERSION 0.1.0)
set(${NAME}_VERSION_ABI 1)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake
                              ${CMAKE_SOURCE_DIR}/CMake/common)
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/CMake/common/Common.cmake)
  message(FATAL_ERROR "CMake/common missing, run: git submodule update --init")
endif()

include(Common)

common_find_package(Brayns REQUIRED)
common_find_package(DCMTK REQUIRED)
common_find_package_post()

set(${NAME}_SOURCES
  DICOMPlugin.cpp
  DICOMParams.cpp
  DICOMLoader.cpp
)

set(${NAME}_PUBLIC_HEADERS
  DICOMPlugin.h
  DICOMParams.h
  DICOMLoader.h
)

set(${NAME}_LINK_LIBRARIES
  braynsParameters braynsPluginAPI vmmlib ${DCMTK_LIBRARIES})

common_library(dicom)
