#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from phm_task_plan.srv import *
from std_msgs.msg import Int32, Float64, String


# Yaml Filedaki Moduller
def module_array():
    system_module = dict(rospy.get_param('~System**'))
    return list(system_module.keys())


# Yaml Filedaki Modullerin Componentleri
def component_array():
    system_component = dict(rospy.get_param('~System**'))
    return list(system_component.values())


# ------------------------------------------------------------------------------------------

def get_parameters_info_dict():
    read_dict = dict(rospy.get_param('~Info**'))

    return read_dict


def get_parameters_type_dict():
    read_dict = dict(rospy.get_param('~Type**'))

    return read_dict


def get_select_parameter_dict(temp_dict, temp_module, temp_component):
    read_dict = dict(temp_dict[str(temp_module + "*")][temp_component])

    return read_dict


def get_select_module_type(temp_dict, temp_module):
    module_type = str(temp_dict[str(temp_module + "*")]["type*"])

    return module_type


def get_wheel_count():
    wheel_count = int(rospy.get_param('~Wheel_Count'))

    return wheel_count


# ------------------------------------------------------------------------------------------

def object_failure_rate_calculation(temp_dict, temp_type):
    if temp_type == 'Serial':
        return object_serial_failure_rate_calculation(temp_dict)

    else:
        return object_parallel_failure_rate_calculation(temp_dict)


# ------------------------------------------------------------------------------------------

# Parallel ise Count u ( 1 + 1/2 + 1/3 + ... 1/ObjectCount) * FailureRate ile çarp
def object_parallel_failure_rate_calculation(temp_dict):
    result = float(temp_dict["ObjectFailureRate"]) * float(parallel_count_calculate(int(temp_dict["ObjectCount"])))

    return result


def object_serial_failure_rate_calculation(temp_dict):
    result = float(temp_dict["ObjectFailureRate"]) * float(temp_dict["ObjectCount"])

    return result

# ------------------------------------------------------------------------------------------


def component_parallel_failure_rate_calculation(temp_list):
    failure_rate_of_parallel_value = 0.0

    if len(temp_list) > 0:
        for i in range(len(temp_list)):
            failure_rate_of_parallel_value += temp_list[i]

        result = float(failure_rate_of_parallel_value) * float(parallel_count_calculate(len(temp_list)))

        return float(result)

    else:
        return float(failure_rate_of_parallel_value)


def component_serial_failure_rate_calculation(temp_list):
    failure_rate_of_serial_value = 0.0

    if len(temp_list) > 0:
        for i in range(len(temp_list)):
            failure_rate_of_serial_value += temp_list[i]

        result = float(failure_rate_of_serial_value)

        return float(result)

    else:
        return float(failure_rate_of_serial_value)

# ------------------------------------------------------------------------------------------


def reliability_parallel_formula(value):
    return float(float(1) - float(value))


def reliability_parallel_calculation(parallel_list):
    calculated_parallel_system_reliability = 1.0

    if len(parallel_list) > 0:
        for i in range(len(parallel_list)):
            calculated_parallel_system_reliability *= parallel_list[i]

        return float(calculated_parallel_system_reliability)

    else:
        return float(calculated_parallel_system_reliability)


def reliability_serial_calculation(serial_list):
    calculated_series_system_reliability = 1.0

    if len(serial_list) > 0:
        for i in range(len(serial_list)):
            calculated_series_system_reliability *= serial_list[i]

        return float(calculated_series_system_reliability)

    else:
        return float(calculated_series_system_reliability)

# ------------------------------------------------------------------------------------------


# λ0      Modülün Hata Oranını Hesaplama
def failure_rate_for_modules(reliability_module, component_list, info_dict, type_dict, graph_control):
    failure_rate_for_module_list = list()

    for i in range(len(component_list)):
        failure_rate_of_series_value = 0.0
        failure_rate_of_parallel_value = 0.0
        failure_rate_of_parallel_component_count = 0

        component_serial_list = list()
        component_parallel_list = list()

        graph_dict = dict()

        for j in range(len(component_list[i])):
            temp_dict = dict()
            read_dict = get_select_parameter_dict(info_dict, reliability_module[i], component_list[i][j])
            temp_component_type = get_select_parameter_dict(type_dict, reliability_module[i], component_list[i][j])
            component_type = temp_component_type["type"]
            temp_dict = read_dict

            if component_type == 'Serial':       # Component Seri ise  Topluyor  -> hata oranı ve miktarı çarpıyor.
                temp_serial_value = float(object_failure_rate_calculation(
                                                                                    read_dict,
                                                                                    component_type))

                component_serial_list.append(temp_serial_value)
                temp_dict["FailureRate"] = temp_serial_value

            if component_type == 'Parallel':       # Component Paralel ise
                temp_parallel_value = float(object_failure_rate_calculation(
                                                                                        read_dict,
                                                                                        component_type))

                component_parallel_list.append(temp_parallel_value)
                temp_dict["FailureRate"] = temp_parallel_value

            temp_dict["Type"] = str(component_type)
            graph_dict[component_list[i][j]] = dict(temp_dict)

        failure_rate_for_module_value = component_serial_failure_rate_calculation(component_serial_list) \
            + component_parallel_failure_rate_calculation(component_parallel_list)

        failure_rate_for_module_list.append(float(failure_rate_for_module_value))

        temp_system_dict = dict()
        temp_system_dict["Type"] = get_select_module_type(type_dict, reliability_module[i])
        temp_system_dict["ObjectCount"] = ''
        temp_system_dict["ObjectFailureRate"] = ''
        temp_system_dict["FailureRate"] = failure_rate_for_module_value
        graph_dict[str(reliability_module[i])] = dict(temp_system_dict)

        if graph_control:
            graph_view = create_module_graph(str(reliability_module[i]), graph_dict)

    return failure_rate_for_module_list


def nominal_reliability_calculate(task_usage, module_failure_rate):
    reliability = math.exp(float(-1) * float(task_usage) * float(module_failure_rate))

    return float(reliability)


# Hazard Rate Parallel Calculate ( 1 + 1/2 + 1/3 + ... + 1/parallel_count )
def parallel_count_calculate(parallel_count):
    if parallel_count == 0:
        return 1

    else:
        temp = 0.0

        for i in range(parallel_count):
            temp = temp + 1/(float(i)+1)

        return pow(temp, (-1))


def percentage_calculated(value):
    return round(float(value) * 100, 6)


def reliability_calculation_function(
                                        type_dict,
                                        reliability_module,
                                        module_usage_during_task_list,
                                        module_failure_rate,
                                        want_a_string,
                                        graph_control):
    temp_reliability_module = reliability_module
    temp_module_usage_during_task_list = module_usage_during_task_list
    temp_module_failure_rate = module_failure_rate

    wheel_count = 1
    temp_string = ''
    calculated_system_nominal_reliability = 1.0         # System Reliability    ( Rsystem )
    calculated_series_system_reliability = 1.0          # Series Reliability    ( Rs = R1 * R2 * ... * Rn )
    calculated_parallel_system_reliability = 1.0        # Parallel Reliability  ( Rp = 1 - Rpi )
    # Rpi = (1-R1) * (1-R2) * ... * (1-Rn)
    reliability_serial_list = list()
    reliability_parallel_list = list()

    graph_dict = dict()

    for i in range(len(temp_reliability_module)):
        temp_dict = dict()
        module_type = get_select_module_type(type_dict, reliability_module[i])

        if temp_reliability_module[i] == 'Mobility':    # Mobility de Tekerlek sayısına bakıyoruz

            mobility_reliability = pow(
                                        float(nominal_reliability_calculate(
                                                                            temp_module_usage_during_task_list[i],
                                                                            temp_module_failure_rate[i])),
                                        wheel_count)

            temp_string += '\t\tWheel Count = %d \n' % wheel_count

            if wheel_count != 0:
                nominal_reliability = float(mobility_reliability)

            else:
                nominal_reliability = 1.0       # Tekerlek Sayisi 0 sa 1 yaparak Çarpımda etkisiz yapıyoruz.

        else:
            nominal_reliability = float(nominal_reliability_calculate(
                                                                        temp_module_usage_during_task_list[i],
                                                                        temp_module_failure_rate[i]))

        # %100 lik reliability sonucu
        percentage_nominal_reliability = float(percentage_calculated(float(nominal_reliability)))
        temp_string += 'Modul Name = %s \tModul Value = %d \nFailure Rate = %.10f \tCalculate = %%%f \n' \
            % (
                temp_reliability_module[i],
                temp_module_usage_during_task_list[i],
                temp_module_failure_rate[i],
                percentage_nominal_reliability)

        temp_string += '---------------------------------------\n\n'

        if module_type == 'Serial':
            # Series Reliability = Rs = R1 * R2 * ... * Rn
            reliability_serial_list.append(float(nominal_reliability))

        if module_type == 'Parallel':
            # Parallel Reliability = Rpi = (1-R1) * (1-R2) * ... * (1-Rn)
            reliability_parallel_list.append(reliability_parallel_formula(nominal_reliability))

        temp_dict["Type"] = str(module_type)
        temp_dict["ModuleValue"] = temp_module_usage_during_task_list[i]
        temp_dict["FailureRate"] = temp_module_failure_rate[i]
        temp_dict["Reliability"] = percentage_nominal_reliability
        graph_dict[temp_reliability_module[i]] = dict(temp_dict)

    calculated_series_system_reliability = reliability_serial_calculation(reliability_serial_list)
    calculated_parallel_system_reliability = reliability_parallel_calculation(reliability_parallel_list)

    if calculated_parallel_system_reliability == 1.0:
        calculated_parallel_system_reliability = 0

    calculated_system_nominal_reliability = float(calculated_series_system_reliability) \
        * float(reliability_parallel_formula(calculated_parallel_system_reliability))
    # Rsystem = Rs * (1-Rpi)

    percentage_system_nominal_reliability = float(percentage_calculated(calculated_system_nominal_reliability))
    temp_string += '\t\t System Reliability = %%%.10f \n\n' % (float(percentage_system_nominal_reliability))
    temp_string += '#####################################################################\n\n'
    temp_system_dict = dict()
    temp_system_dict["Type"] = ''
    temp_system_dict["ModuleValue"] = ''
    temp_system_dict["FailureRate"] = ''
    temp_system_dict["Reliability"] = percentage_system_nominal_reliability

    module_name = "System"
    graph_dict[str(module_name)] = dict(temp_system_dict)

    if graph_control:
        graph_view = create_system_graph(module_name, graph_dict)

    if want_a_string:
        return temp_string

    else:
        return float(percentage_system_nominal_reliability)

# ------------------------------------------------------------------------------------------


def create_system_graph(module_name, graph_dict):
    import pydot

    module_list = list(graph_dict.keys())

    graph = pydot.Dot(graph_type='digraph')
    Colors = ["red", "orange", "yellow", "green", "#0000ff", "#976856"]

    system = dict()

    temp_write = str(module_name) + "\nReliability = " + str(graph_dict[str(module_name)]["Reliability"])
    top_system = pydot.Node(temp_write, style="filled", fillcolor=Colors[4], shape="box")

    for i in range(len(module_list)):
        if str(module_list[i]) == module_name:
            continue

        else:
            module_type = str(graph_dict[str(module_list[i])]["Type"])

            if module_type == 'Serial':
                color = Colors[1]

            else:
                color = Colors[3]

            write = module_list[i] + "\nReliability = " + str(graph_dict[str(module_list[i])]["Reliability"]) \
                + "\n\nModule Value = " + str(graph_dict[str(module_list[i])]["ModuleValue"]) \
                + "\nFailure Rate = " + str(graph_dict[str(module_list[i])]["FailureRate"]) + "\n"

            system[str(i)] = pydot.Node(str(write), style="filled", fillcolor=color, shape="box")
            graph.add_node(system[str(i)])

    for i in range(len(module_list)):
        if str(module_list[i]) == module_name:
            continue
        else:
            graph.add_edge(pydot.Edge(top_system, system[str(i)]))

    image_name = module_name + "_Reliability"
    write_name = '/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/source/' + str(image_name) + '.png'

    graph.write_png(str(write_name))

    return True


def create_module_graph(module_name, graph_dict):
    import pydot

    module_list = list(graph_dict.keys())

    graph = pydot.Dot(graph_type='digraph')
    Colors = ["red", "orange", "yellow", "green", "#0000ff", "#976856"]

    system = dict()

    temp_write = str(module_name) + "\nFailure Rate = " + str(graph_dict[str(module_name)]["FailureRate"]) \
        + "\n\nType = " + str(graph_dict[str(module_name)]["Type"])

    top_module = pydot.Node(temp_write, style="filled", fillcolor=Colors[4])

    for i in range(len(module_list)):
        if str(module_list[i]) == module_name:
            continue

        else:
            module_type = str(graph_dict[str(module_list[i])]["Type"])

            if module_type == 'Serial':
                color = Colors[1]

            else:
                color = Colors[3]

            write = module_list[i] + "\nFailure Rate = " + str(graph_dict[str(module_list[i])]["FailureRate"]) \
                + "\n\nObject Count = " + str(graph_dict[str(module_list[i])]["ObjectCount"]) \
                + "\nObject Failure Rate = " + str(graph_dict[str(module_list[i])]["ObjectFailureRate"]) + "\n"

            system[str(i)] = pydot.Node(str(write), style="filled", fillcolor=color, shape="house")
            graph.add_node(system[str(i)])

    for i in range(len(module_list)):
        if str(module_list[i]) == module_name:
            continue

        else:
            graph.add_edge(pydot.Edge(top_module, system[str(i)]))

    image_name = str(module_name)
    image_name = image_name.replace(" ", "_")
    write_name = '/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/source/module/' \
        + str(image_name) + '.png'

    graph.write_png(str(write_name))

    return True

# ------------------------------------------------------------------------------------------


def task_plan_client(module):
    rospy.wait_for_service('task_service')

    try:
        # create a handle to the task_plan service
        task_plan = rospy.ServiceProxy('task_service', TaskService)

        # formal style
        response = task_plan.call(TaskServiceRequest(module))

        return response.task
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def task_plan_client_call(value):
    return task_plan_client(value)


def module_usage_during_task():
    temp_list = list()

    for i in range(len(reliability_module)):
        temp_list.append(task_plan_client_call(reliability_module[i]))

    return temp_list

# ------------------------------------------------------------------------------------------


def monitoring(want_a_string):

    if want_a_string:
        return show_value_string

    else:
        return show_value_float

# ------------------------------------------------------------------------------------------


def reliability_publisher():
    # rospy.init_node('reliability_node')
    # pub=rospy.Publisher('Deneme_Background', Float64, queue_size=10)
    rate = rospy.Rate(2)

    # kontrol = 0
    graph_control = True
    value_string_control = True
    info_dict = get_parameters_info_dict()
    type_dict = get_parameters_type_dict()
    # wheel_count = get_wheel_count()

    module_failure_rate = failure_rate_for_modules(
                                                    reliability_module,
                                                    component_list,
                                                    info_dict,
                                                    type_dict,
                                                    graph_control)
    while not rospy.is_shutdown():

        show_value_string = reliability_calculation_function(
                                                                type_dict,
                                                                reliability_module,
                                                                module_usage_during_task_list,
                                                                module_failure_rate,
                                                                value_string_control,
                                                                graph_control)

        print ("%s" % (show_value_string))
        graph_control = False

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('reliability_node')
        global reliability_module
        global component_list
        reliability_module = module_array()
        component_list = component_array()

        module_usage_during_task_list = module_usage_during_task()
        reliability_publisher()
    except rospy.ROSInterruptException:
        pass
