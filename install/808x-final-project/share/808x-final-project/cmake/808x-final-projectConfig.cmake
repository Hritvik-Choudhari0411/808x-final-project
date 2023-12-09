# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_808x-final-project_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED 808x-final-project_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(808x-final-project_FOUND FALSE)
  elseif(NOT 808x-final-project_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(808x-final-project_FOUND FALSE)
  endif()
  return()
endif()
set(_808x-final-project_CONFIG_INCLUDED TRUE)

# output package information
if(NOT 808x-final-project_FIND_QUIETLY)
  message(STATUS "Found 808x-final-project: 0.0.0 (${808x-final-project_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package '808x-final-project' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${808x-final-project_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(808x-final-project_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${808x-final-project_DIR}/${_extra}")
endforeach()
