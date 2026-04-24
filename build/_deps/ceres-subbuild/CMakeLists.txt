# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.3)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(ceres-populate NONE)

include(ExternalProject)
ExternalProject_Add(ceres-populate
                     "UPDATE_DISCONNECTED" "False" "SYSTEM" "URL" "https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz"
                    SOURCE_DIR          "/home/user/catkin_ws/build/_deps/ceres-src"
                    BINARY_DIR          "/home/user/catkin_ws/build/_deps/ceres-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
