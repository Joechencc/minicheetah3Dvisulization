add_executable(example_topic_republisher example_topic_republisher.cpp)
target_link_libraries( example_topic_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(example_topic_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(other_topic_republisher other_topic_republisher.cpp)
target_link_libraries( other_topic_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(other_topic_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(leg_control_data_republisher leg_control_data_republisher.cpp)
target_link_libraries( leg_control_data_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(leg_control_data_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


