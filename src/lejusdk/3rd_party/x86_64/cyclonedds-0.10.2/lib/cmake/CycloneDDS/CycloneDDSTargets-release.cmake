#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CycloneDDS::idlc" for configuration "Release"
set_property(TARGET CycloneDDS::idlc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(CycloneDDS::idlc PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/idlc"
  )

list(APPEND _IMPORT_CHECK_TARGETS CycloneDDS::idlc )
list(APPEND _IMPORT_CHECK_FILES_FOR_CycloneDDS::idlc "${_IMPORT_PREFIX}/bin/idlc" )

# Import target "CycloneDDS::idl" for configuration "Release"
set_property(TARGET CycloneDDS::idl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(CycloneDDS::idl PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcycloneddsidl.so.0.10.2"
  IMPORTED_SONAME_RELEASE "libcycloneddsidl.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS CycloneDDS::idl )
list(APPEND _IMPORT_CHECK_FILES_FOR_CycloneDDS::idl "${_IMPORT_PREFIX}/lib/libcycloneddsidl.so.0.10.2" )

# Import target "CycloneDDS::ddsc" for configuration "Release"
set_property(TARGET CycloneDDS::ddsc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(CycloneDDS::ddsc PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libddsc.so.0.10.2"
  IMPORTED_SONAME_RELEASE "libddsc.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS CycloneDDS::ddsc )
list(APPEND _IMPORT_CHECK_FILES_FOR_CycloneDDS::ddsc "${_IMPORT_PREFIX}/lib/libddsc.so.0.10.2" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
