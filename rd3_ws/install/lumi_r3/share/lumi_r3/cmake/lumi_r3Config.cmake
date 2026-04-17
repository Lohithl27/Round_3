# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lumi_r3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lumi_r3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lumi_r3_FOUND FALSE)
  elseif(NOT lumi_r3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lumi_r3_FOUND FALSE)
  endif()
  return()
endif()
set(_lumi_r3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lumi_r3_FIND_QUIETLY)
  message(STATUS "Found lumi_r3: 1.0.0 (${lumi_r3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lumi_r3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${lumi_r3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lumi_r3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${lumi_r3_DIR}/${_extra}")
endforeach()
