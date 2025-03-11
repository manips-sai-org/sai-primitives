# Install script for directory: /home/src0/OpenSai/core/sai2-primitives/examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/01-joint_control/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/02-joint_control_internal_otg/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/03-cartesian_motion_control/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/04-task_and_redundancy/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/05-using_robot_controller/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/06-partial_joint_task/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/07-surface_surface_contact/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/08-partial_motion_force_task/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/09-3d_position_force_controller/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/10-3d_orientation_controller/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/11-planar_robot_controller/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/15-haptic_control_impedance_type/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/16-haptic_control_admittance_type/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/17-bilateral_teleop_with_POPC/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/18-panda_singularity/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/19-puma_singularity/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/97-self_collision/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/98-nullspace/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/99-apf/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/100-joint_limits/cmake_install.cmake")
  include("/home/src0/OpenSai/core/sai2-primitives/openGJK/examples/101-collision/cmake_install.cmake")

endif()

