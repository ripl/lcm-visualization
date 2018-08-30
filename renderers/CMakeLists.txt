cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME renderers)
project(hr-renderer)

# The BASE_PATH is where the models, data, config, etc. directories are located
if (NOT BASE_PATH)
    set(BASE_PATH ${CMAKE_INSTALL_PREFIX})
else (NOT EXISTS ${BASE_PATH})
    message(WARNING "${BASE_PATH} is not a valid directory. Setting BASE_PATH: ${CMAKE_INSTALL_PREFIX}")
    set(BASE_PATH ${CMAKE_INSTALL_PREFIX})
endif()

include(cmake/pods.cmake)
include(cmake/models.cmake)

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)

list(APPEND OPENGL_LIBRARIES GL)
set(GLUT_CFLAGS "")
set(GLUT_LIBRARIES -lglut)

# Copy the models over to BUILD_PREFIX
pods_install_model_directories(models/)

pkg_check_modules(BOT_CORE REQUIRED bot2-core)
pkg_check_modules(BOT_VIS REQUIRED bot2-vis)
pkg_check_modules(BOT_FRAMES REQUIRED bot2-frames)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(OPENCV REQUIRED opencv)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(GLIB2 REQUIRED glib-2.0)

add_subdirectory(src)
