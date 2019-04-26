cmake_minimum_required(VERSION 2.8.3)
project(phm_mission_analyze)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  phm_task_plan
  message_generation
  roslint
  rosunit
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  phm_task_plan
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs phm_task_plan)
roslint_python(src/phm_mission_analyze/mission_analyze_node.py)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch src test)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()


catkin_add_nosetests(test/test_mission_analyze_node.py)