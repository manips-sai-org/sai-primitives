set(EXAMPLE_NAME 04-task_and_redundancy)
# create an executable
add_executable(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)

# and link the library against the executable
target_link_libraries(${EXAMPLE_NAME} ${SAI-PRIMITIVES_LIBRARIES}
                      ${SAI-PRIMITIVES_EXAMPLES_COMMON_LIBRARIES})
