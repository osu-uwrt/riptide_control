#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "FaultControllerModel::FaultControllerModel" for configuration ""
set_property(TARGET FaultControllerModel::FaultControllerModel APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(FaultControllerModel::FaultControllerModel PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/./FaultControllerModel.so"
  IMPORTED_SONAME_NOCONFIG "FaultControllerModel.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS FaultControllerModel::FaultControllerModel )
list(APPEND _IMPORT_CHECK_FILES_FOR_FaultControllerModel::FaultControllerModel "${_IMPORT_PREFIX}/./FaultControllerModel.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
