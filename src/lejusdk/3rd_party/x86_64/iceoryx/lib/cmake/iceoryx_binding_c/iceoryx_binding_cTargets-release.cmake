#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "iceoryx_binding_c::iceoryx_binding_c" for configuration "Release"
set_property(TARGET iceoryx_binding_c::iceoryx_binding_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(iceoryx_binding_c::iceoryx_binding_c PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libiceoryx_binding_c.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS iceoryx_binding_c::iceoryx_binding_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_iceoryx_binding_c::iceoryx_binding_c "${_IMPORT_PREFIX}/lib/libiceoryx_binding_c.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
