set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(sai_primitives_py SHARED bindings.cpp)
target_compile_features(sai_primitives_py PRIVATE cxx_std_17)
target_link_libraries(sai_primitives_py PRIVATE pybind11::headers Python::Python)
if(APPLE)
  target_link_options(
    sai_primitives_py PRIVATE "-Wl,-flat_namespace" "-Wl,-undefined,dynamic_lookup")
endif()

target_link_libraries(
  sai_primitives_py
  PRIVATE sai-primitives ${SAI-MODEL_LIBRARIES} ${RUCKIG_LIBRARIES}
          ${OPENGJK_LIBRARIES} ${NLOPT_LIBRARIES})

target_include_directories(
  sai_primitives_py
  PRIVATE ${SAI-PRIMITIVES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
          ${SAI-MODEL_INCLUDE_DIRS} ${RUCKIG_INCLUDE_DIR}
          ${OPENGJK_INCLUDE_DIR} ${NLOPT_INCLUDE_DIR})

set_target_properties(
  sai_primitives_py
  PROPERTIES PREFIX "" SUFFIX ".${Python_SOABI}.so"
             LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/python")
