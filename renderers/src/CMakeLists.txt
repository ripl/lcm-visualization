add_definitions(-DBASE_PATH="${BASE_PATH}")
add_definitions(-std=gnu99)

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${LCMTYPES_INCLUDE_DIRS}
)

file(GLOB C_FILES *.c)
file(GLOB H_FILES *.h)


# Create a shared library libquad-renderers.so with a single source file
add_library(husky-renderers SHARED ${C_FILES})

set(REQUIRED_LIBRARIES
    lcm
    bot2-core
    bot2-vis
    lcmtypes_bot2-procman
    lcmtypes_ripl-general
    lcmtypes_rrtstar
    lcmtypes_obstacle-detector
    lcmtypes_gridmap
    common-utils
    gridmap-utils
    opencv
    jpeg-utils
    bot2-lcmgl-renderer
    )

pods_use_pkg_config_packages(husky-renderers ${REQUIRED_LIBRARIES})

# make the header public
# install it to include/renderers
pods_install_headers(${H_FILES} DESTINATION husky_renderers)

# make the library public
pods_install_libraries(husky-renderers)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(husky-renderers
    CFLAGS
    LIBS -lhusky-renderers
    REQUIRES ${REQUIRED_LIBRARIES}
    VERSION 0.0.1)
