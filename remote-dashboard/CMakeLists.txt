cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME remote-dashboard)
include(cmake/pods.cmake)

pods_install_python_script(remote-dashboard-server src/server/server.py)
