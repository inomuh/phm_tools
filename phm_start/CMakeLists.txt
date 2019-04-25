cmake_minimum_required(VERSION 2.8.3)
project(phm_start)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  rosunit
)


catkin_python_setup()

catkin_package()

roslint_python(src/phm_start/phm_gui.py)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   src/phm_start/phm_gui.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch src test)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

catkin_add_nosetests(test/test_phm_gui.py)