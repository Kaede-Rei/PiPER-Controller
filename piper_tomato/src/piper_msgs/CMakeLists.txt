cmake_minimum_required(VERSION 3.0.2)
project(piper_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
    actionlib_msgs
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
    FILES
    
)

add_action_files(
    FILES

)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    geometry_msgs 
    std_msgs 
    actionlib_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
