# Additional macros to simplify compliance with the pods build policies.
#
# To enable the macros, add the following lines to CMakeLists.txt:
#   set(POD_NAME <pod-name>)
#   include(cmake/pods.cmake)
#
# If POD_NAME is not set, then the CMake source directory is used as POD_NAME
#
# Next, any of the following macros can be used.  See the individual macro
# definitions in this file for individual documentation.
#
# C/C++
#   pods_install_headers(...)
#   pods_install_libraries(...)
#   pods_install_executables(...)
#   pods_install_pkg_config_file(...)
#
#   pods_use_pkg_config_packages(...)
#
# Python
#   pods_install_python_packages(...)
#   pods_install_python_script(...)
#
# Java
#   None yet
#
# ----
# File: pods.cmake
# Distributed with pods version: 12.09.21

# pods_install_headers(<header1.h> ... DESTINATION <subdir_name>)
#
# Install a (list) of header files.
#
# Header files will all be installed to include/<subdir_name>
#
# example:
#   add_library(perception detector.h sensor.h)
#   pods_install_headers(detector.h sensor.h DESTINATION perception)
#

function(pods_install_model_directories)
    install(DIRECTORY ${ARGV} DESTINATION models)
endfunction(pods_install_model_directories)
