add_definitions(
    -std=gnu99)

#build the viewer

add_executable(hr-viewer 
    udp_util.c
    main.cpp
    )

#add_executable(hr-full-viewer 
#    udp_util.c
#    main_full.cpp
#    )

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
    hr-renderers
    laser-util-renderer
    image-util-renderer
    lcmtypes_hr-lcmtypes
    map3d_interface
    hr-path-util
    #velodyne-renderer
    image-util-renderer
    occ-map-renderers
    kinect-renderer
    #collections_renderer  
    octomap-renderer
    )

pods_install_executables(hr-viewer)