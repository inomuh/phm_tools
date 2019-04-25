#! /usr/bin/env python

import rospy
from phm_task_plan.srv import *


def mission_analyze(module):
    rospy.wait_for_service('task_service')

    try:
        # create a handle to the task_plan service
        task_plan = rospy.ServiceProxy('task_service', TaskService)

        # Ekrana Yazdiriyo
        print "Requesting %s" % (module)

        # simplified style
        resp1 = task_plan(module)

        # formal style
        resp2 = task_plan.call(TaskServiceRequest(module))

        return resp2.task
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def task_plan_client():
    module = 'Power'
    task = mission_analyze(module)

    print "Task = %d" % (task)

if __name__ == "__main__":
    task_plan_client()
