set(EXAMPLE_NAME 98-nullspace)
# create an executable
add_executable(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)
add_executable(98-joints 98-joints.cpp)

# and link the library against the executable
target_link_libraries(${EXAMPLE_NAME} ${SAI2-PRIMITIVES_LIBRARIES}
                      ${SAI2-PRIMITIVES_EXAMPLES_COMMON_LIBRARIES})

target_link_libraries(98-joints ${SAI2-PRIMITIVES_LIBRARIES}
                      ${SAI2-PRIMITIVES_EXAMPLES_COMMON_LIBRARIES})
