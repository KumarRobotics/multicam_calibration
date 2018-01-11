#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ceres" for configuration "None"
set_property(TARGET ceres APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(ceres PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "/usr/lib/x86_64-linux-gnu/libglog.so"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libceres.so.1.11.0"
  IMPORTED_SONAME_NONE "libceres.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS ceres )
list(APPEND _IMPORT_CHECK_FILES_FOR_ceres "${_IMPORT_PREFIX}/lib/libceres.so.1.11.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
