#!/usr/bin/env python

# import the AddTwoInts service
from phm_task_plan.srv import *
import rospy


def task_plan_func(req):
    module = req.module
    rospy.init_node('task_plan_node')

    task = rospy.get_param('~' + module)
    return TaskServiceResponse(task)


def task_plan():
    rospy.init_node('task_plan_node')
    s = rospy.Service('task_service', TaskService, task_plan_func)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    task_plan()
