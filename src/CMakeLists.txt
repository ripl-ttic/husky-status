add_definitions(
#    -ggdb3
    -std=gnu99
    )

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${LCMTYPES_INCLUDE_DIRS}
    )

add_executable(husky-status main.c)

pods_use_pkg_config_packages(husky-status
    glib-2.0
    gthread-2.0
    lcm
    bot2-core
    lcmtypes_bot2-core
    lcmtypes_ros2lcm_bridge
    lcmtypes_robot-status
    bot2-param-client)


pods_install_executables(husky-status)
