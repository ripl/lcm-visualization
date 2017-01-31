cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME renderers)
project(hr-renderer)

include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)

list(APPEND OPENGL_LIBRARIES GL)
set(GLUT_CFLAGS "")
set(GLUT_LIBRARIES -lglut)

pkg_check_modules(BOT_CORE REQUIRED bot2-core)
pkg_check_modules(BOT_VIS REQUIRED bot2-vis)
pkg_check_modules(BOT_FRAMES REQUIRED bot2-frames)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(OPENCV REQUIRED opencv)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(GLIB2 REQUIRED glib-2.0)

add_subdirectory(src)
