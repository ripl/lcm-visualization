add_definitions(
    -std=gnu99)

add_executable(hr-viewer
    udp_util.c
    main.cpp
    )

include_directories(
    ${GTK2_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIR})

target_link_libraries(hr-viewer
    ${GTK2_LDFLAGS}
    ${GLUT_LIBRARIES})

pods_use_pkg_config_packages(hr-viewer
    bot2-vis
    bot2-lcmgl-renderer
    bot2-frames-renderers
    bot2-frames
    opencv
    husky-renderers
    laser-utils-renderer
    image-utils-renderer
    lcmtypes_hr-lcmtypes
    lcmtypes_map
    map3d_interface
    occ-map-renderers
    octomap-renderer
    rrtstar-renderer
    obstacle-renderer
    robot-status-renderer
    )

pods_install_executables(hr-viewer)
