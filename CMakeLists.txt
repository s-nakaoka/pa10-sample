option(BUILD_PA10_SAMPLE "Building the PA10 sample" OFF)

if(NOT BUILD_PA10_SAMPLE)
  return()
endif()

if(UNIX)
  add_custom_target(
    symlink_to_pa10_sample ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/share ${CNOID_BINARY_SHARE_DIR}/pa10)
endif()

install(DIRECTORY share/ DESTINATION ${CNOID_SHARE_SUBDIR}/pa10)

add_subdirectory(src)
