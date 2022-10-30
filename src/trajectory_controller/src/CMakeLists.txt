add_executable(move_group_interface move_group_interface.cpp)

target_link_libraries(
        move_group_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_executable(generate_trajectory generate_trajectory.cpp)

target_link_libraries(
        generate_trajectory
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        tinyxml2
)

install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS generate_trajectory DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})