#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float32, Int32
import random


class SensorsPub:
    def __init__(self):
        self.time = 0

        self.publisher()


    def create_temperature_number(self):
        result = random.randint(20, 159)

        return result

    def random_float(self, min_number, max_number):
        result = round(random.uniform(min_number, max_number), 3)

        return result

    def create_failure_rate(self, e_count):
        number = round(random.uniform(0, 1), 3)

        failure_rate = str(str(number) + "e-" + str(e_count))

        return failure_rate

    def publisher(self):
        self.publisher_time = rospy.Publisher('/sensor_time', Int32, queue_size=10)
        self.publisher_capacitor_temperature = rospy.Publisher('/capacitor_temperature', Float32, queue_size=10)
        self.publisher_temperature_sensor_1 = rospy.Publisher('/temperature_sensor_1', Float32, queue_size=10)
        self.publisher_temperature_sensor_2 = rospy.Publisher('/temperature_sensor_2', Float32, queue_size=10)
        self.publisher_temperature_sensor_3 = rospy.Publisher('/temperature_sensor_3', Float32, queue_size=10)
        self.publisher_temperature_sensor_4 = rospy.Publisher('/temperature_sensor_4', Float32, queue_size=10)
        self.publisher_temperature_sensor_5 = rospy.Publisher('/temperature_sensor_5', Float32, queue_size=10)
        self.publisher_temperature_sensor_6 = rospy.Publisher('/temperature_sensor_6', Float32, queue_size=10)
        self.publisher_temperature_sensor_7 = rospy.Publisher('/temperature_sensor_7', Float32, queue_size=10)
        self.publisher_temperature_sensor_8 = rospy.Publisher('/temperature_sensor_8', Float32, queue_size=10)
        self.publisher_temperature_sensor_9 = rospy.Publisher('/temperature_sensor_9', Float32, queue_size=10)
        self.publisher_random_1 = rospy.Publisher('/sensor_topic_1', String, queue_size=10)
        self.publisher_random_2 = rospy.Publisher('/sensor_topic_2', String, queue_size=10)
        self.publisher_random_3 = rospy.Publisher('/sensor_topic_3', String, queue_size=10)
        self.publisher_random_4 = rospy.Publisher('/sensor_topic_4', String, queue_size=10)
        self.publisher_random_5 = rospy.Publisher('/sensor_topic_5', String, queue_size=10)

        self.rate = rospy.Rate(2)

        msg_time = Int32()
        msg_capacitor_temperature = Float32()
        msg_random_1 = String()
        msg_random_2 = String()
        msg_random_3 = String()
        msg_random_4 = String()
        msg_random_5 = String()

        msg_temperature_sensor_1 = Float32()
        msg_temperature_sensor_2 = Float32()
        msg_temperature_sensor_3 = Float32()
        msg_temperature_sensor_4 = Float32()
        msg_temperature_sensor_5 = Float32()
        msg_temperature_sensor_6 = Float32()
        msg_temperature_sensor_7 = Float32()
        msg_temperature_sensor_8 = Float32()
        msg_temperature_sensor_9 = Float32()

        counter = 0
        while not rospy.is_shutdown():
            counter += 1
            msg_time.data = int(counter)
            self.publisher_time.publish(msg_time)

            msg_capacitor_temperature.data = float(self.create_temperature_number())
            self.publisher_capacitor_temperature.publish(msg_capacitor_temperature)

            msg_random_1.data = str(self.create_failure_rate(4))
            msg_random_2.data = str(self.create_failure_rate(5))
            msg_random_3.data = str(self.create_failure_rate(6))
            msg_random_4.data = str(self.create_failure_rate(7))
            msg_random_5.data = str(self.create_failure_rate(8))

            self.publisher_random_1.publish(msg_random_1)
            self.publisher_random_2.publish(msg_random_2)
            self.publisher_random_3.publish(msg_random_3)
            self.publisher_random_4.publish(msg_random_4)
            self.publisher_random_5.publish(msg_random_5)


            msg_temperature_sensor_1.data = float(self.create_temperature_number())
            msg_temperature_sensor_2.data = float(self.create_temperature_number())
            msg_temperature_sensor_3.data = float(self.create_temperature_number())
            msg_temperature_sensor_4.data = float(self.create_temperature_number())
            msg_temperature_sensor_5.data = float(self.create_temperature_number())
            msg_temperature_sensor_6.data = float(self.create_temperature_number())
            msg_temperature_sensor_7.data = float(self.create_temperature_number())
            msg_temperature_sensor_8.data = float(self.create_temperature_number())
            msg_temperature_sensor_9.data = float(self.create_temperature_number())

            self.publisher_temperature_sensor_1.publish(msg_temperature_sensor_1)
            self.publisher_temperature_sensor_2.publish(msg_temperature_sensor_2)
            self.publisher_temperature_sensor_3.publish(msg_temperature_sensor_3)
            self.publisher_temperature_sensor_4.publish(msg_temperature_sensor_4)
            self.publisher_temperature_sensor_5.publish(msg_temperature_sensor_5)
            self.publisher_temperature_sensor_6.publish(msg_temperature_sensor_6)
            self.publisher_temperature_sensor_7.publish(msg_temperature_sensor_7)
            self.publisher_temperature_sensor_8.publish(msg_temperature_sensor_8)
            self.publisher_temperature_sensor_9.publish(msg_temperature_sensor_9)

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('sensor_deneme_publisher')

    SensorsPub()