cmake_minimum_required(VERSION 2.8.3)
project(phm_hazard_rate_calculation)

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


foreach(dir launch params src test)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

#catkin_add_nosetests(test/test_class_failure_rate_calculation.py)
#catkin_add_nosetests(test/test_class_electrical_equipment.py)
#catkin_add_nosetests(test/test_class_mechanical_equipment.py)
#catkin_add_nosetests(test/test_phm_hazard_rate_calculation_node.py)

