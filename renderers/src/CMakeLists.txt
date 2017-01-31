add_definitions(-std=gnu99)

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${LCMTYPES_INCLUDE_DIRS}
)

# Create a shared library libquad-renderers.so with a single source file
add_library(hr-renderers SHARED
    er_gl_utils.c
    viewer_aux_data.c
    tile_set.c
    renderer_waypoint.c
    renderer_rrtstar.c
    renderer_occupancy_map.c
    renderer_grid.c
    renderer_localize.c
    #renderer_navigator_plan.c
    renderer_tracks.c
    renderer_robot_wavefront_model.c
    renderer_quad_model.c
    renderer_simobs.c
    renderer_gridmap.c
    renderer_person_model.c
    #renderer_verify_check_gridmap.c
    renderer_host_status.c
    renderer_sensor_status.c
    renderer_robot_status.c
    #renderer_pcl_points.c
    #drawing_utility.c
    #renderer_robot_commands.c
    renderer_manual_calib.c
    renderer_vision_lcmgl.c
    #renderer_topological_graph.c
    )

#target_link_libraries (hr-renderers
#    ${GTK2_LDFLAGS}
#    ${OPENGL_LIBRARIES}
#    ${GLUT_LIBRARIES}
#    ${LCMTYPES_LIBS})
    
    
set(REQUIRED_LIBRARIES
    lcm
    bot2-core
    bot2-vis
    #bot2-param-client 
    #bot2-frames
    hr-common
    check_gridmap
    lcmtypes_bot2-procman
    lcmtypes_hr-lcmtypes
    lcmtypes_rrtstar
    #map3d_interface
    opencv
    jpeg-utils
    bot2-lcmgl-renderer
    #alcmtypes_bot2-lcmgl
    hr-path-util
    )

pods_use_pkg_config_packages(hr-renderers ${REQUIRED_LIBRARIES})

# make the header public
# install it to include/quad-renderers
pods_install_headers(er_renderers.h er_gl_utils.h viewer_aux_data.h DESTINATION er_renderers)

# make the library public
pods_install_libraries(hr-renderers)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(hr-renderers
    CFLAGS
    LIBS -lhr-renderers 
    REQUIRES ${REQUIRED_LIBRARIES}
    VERSION 0.0.1)