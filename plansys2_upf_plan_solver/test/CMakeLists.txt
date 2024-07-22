set(TEST_PDDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pddl)

install(DIRECTORY
  ${TEST_PDDL_DIR}
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(unit)
