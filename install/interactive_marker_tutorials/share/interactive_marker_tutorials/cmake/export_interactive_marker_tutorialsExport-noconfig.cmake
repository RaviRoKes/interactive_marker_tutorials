#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "interactive_marker_tutorials::basic_controls" for configuration ""
set_property(TARGET interactive_marker_tutorials::basic_controls APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(interactive_marker_tutorials::basic_controls PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libbasic_controls.so"
  IMPORTED_SONAME_NOCONFIG "libbasic_controls.so"
  )

list(APPEND _cmake_import_check_targets interactive_marker_tutorials::basic_controls )
list(APPEND _cmake_import_check_files_for_interactive_marker_tutorials::basic_controls "${_IMPORT_PREFIX}/lib/libbasic_controls.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
