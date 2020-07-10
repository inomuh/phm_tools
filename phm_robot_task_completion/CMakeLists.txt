cmake_minimum_required(VERSION 2.8.3)
project(phm_robot_task_completion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roslint
  rostest
  rosunit
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy std_msgs rostest )



include_directories(
  ${catkin_INCLUDE_DIRS}
)


foreach(dir launch src test)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

#catkin_add_nosetests(test/test_class_robot_task_completion.py)
#catkin_add_nosetests(test/test_phm_robot_task_completion_node.py)
