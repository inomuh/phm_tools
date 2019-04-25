cmake_minimum_required(VERSION 2.8.3)
project(phm_task_plan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslint
  rosunit
)

catkin_python_setup()

## Declare ROS messages and services
add_service_files(FILES TaskService.srv)


generate_messages(DEPENDENCIES std_msgs)

catkin_package()

roslint_python(src/phm_task_plan/task_plan_node.py)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch params src srv test)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()


catkin_add_nosetests(test/test_task_plan_node.py)