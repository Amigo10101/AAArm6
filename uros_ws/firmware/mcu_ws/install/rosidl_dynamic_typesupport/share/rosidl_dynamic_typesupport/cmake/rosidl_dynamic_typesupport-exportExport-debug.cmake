#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rosidl_dynamic_typesupport::rosidl_dynamic_typesupport" for configuration "Debug"
set_property(TARGET rosidl_dynamic_typesupport::rosidl_dynamic_typesupport APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(rosidl_dynamic_typesupport::rosidl_dynamic_typesupport PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/librosidl_dynamic_typesupport.a"
  )

list(APPEND _cmake_import_check_targets rosidl_dynamic_typesupport::rosidl_dynamic_typesupport )
list(APPEND _cmake_import_check_files_for_rosidl_dynamic_typesupport::rosidl_dynamic_typesupport "${_IMPORT_PREFIX}/lib/librosidl_dynamic_typesupport.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
