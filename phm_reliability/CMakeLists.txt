cmake_minimum_required(VERSION 2.8.3)
project(phm_reliability)

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

roslint_python(src/phm_reliability/reliability_node.py)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   src/phm_reliability/reliability_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch params src test)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

catkin_add_nosetests(test/test_reliability_node.py)


#find_package(rostest REQUIRED)
#add_rostest(launch/test_reliability.test)

#if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(launch/test_reliability.test)
#endif()
