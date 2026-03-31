#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kuavo_common::kuavo_common" for configuration "Release"
set_property(TARGET kuavo_common::kuavo_common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(kuavo_common::kuavo_common PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libkuavo_common.so"
  IMPORTED_SONAME_RELEASE "libkuavo_common.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS kuavo_common::kuavo_common )
list(APPEND _IMPORT_CHECK_FILES_FOR_kuavo_common::kuavo_common "${_IMPORT_PREFIX}/lib/libkuavo_common.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
