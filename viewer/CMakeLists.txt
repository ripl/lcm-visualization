cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME viewer)
include(cmake/pods.cmake)

set(GLUT_CFLAGS "")
set(GLUT_LIBRARIES -lglut)


#tell cmake to build these subdirectories
add_subdirectory(src)

