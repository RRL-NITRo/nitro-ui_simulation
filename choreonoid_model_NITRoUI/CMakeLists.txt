# Built as part of Choreonoid after copying this directory to choreonoid/ext.
# Registration files refer to the robot projects as
# ../../choreonoid_model_NITRoUI/project/<name> (relative to WRS2025/project).

if(NOT ENABLE_GUI)
  return()
endif()

if(UNIX)
  add_custom_target(
    symlink_to_choreonoid_model_NITRoUI ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR} ${CHOREONOID_BINARY_SHARE_DIR}/choreonoid_model_NITRoUI)
endif()

install(DIRECTORY
  model
  project
  DESTINATION ${CHOREONOID_SHARE_SUBDIR}/choreonoid_model_NITRoUI)
