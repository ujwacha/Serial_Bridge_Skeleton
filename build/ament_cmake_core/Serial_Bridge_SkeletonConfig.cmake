# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Serial_Bridge_Skeleton_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Serial_Bridge_Skeleton_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Serial_Bridge_Skeleton_FOUND FALSE)
  elseif(NOT Serial_Bridge_Skeleton_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Serial_Bridge_Skeleton_FOUND FALSE)
  endif()
  return()
endif()
set(_Serial_Bridge_Skeleton_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Serial_Bridge_Skeleton_FIND_QUIETLY)
  message(STATUS "Found Serial_Bridge_Skeleton: 0.0.0 (${Serial_Bridge_Skeleton_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Serial_Bridge_Skeleton' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT Serial_Bridge_Skeleton_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Serial_Bridge_Skeleton_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Serial_Bridge_Skeleton_DIR}/${_extra}")
endforeach()
