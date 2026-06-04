set(EXAMPLE_NAME 23-latency_aware_contact_renderer)

add_executable(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)

target_link_libraries(${EXAMPLE_NAME} ${SAI-PRIMITIVES_LIBRARIES}
					  ${SAI-PRIMITIVES_EXAMPLES_COMMON_LIBRARIES})
