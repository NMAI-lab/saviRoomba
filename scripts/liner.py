#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

right_sensor = 8
center_sensor = 10
left_sensor = 12

GPIO.setwarnings(False)
GPIO.setup(right_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(center_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(left_sensor, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def liner():
    pub = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.init_node('liner', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if GPIO.input(center_sensor) == 0 and GPIO.input(right_sensor) == 1 and GPIO.input(left_sensor) == 1:
            line = "position(center)"
        elif GPIO.input(right_sensor) == 0 and GPIO.input(center_sensor) == 1 and GPIO.input(left_sensor) == 1:
            line = "position(right)"
        elif GPIO.input(right_sensor) == 0 and GPIO.input(center_sensor) == 0 and GPIO.input(left_sensor) == 1:
            line = "position(right)"
        elif GPIO.input(left_sensor) == 0 and GPIO.input(right_sensor) == 1 and GPIO.input(center_sensor) == 1:
            line = "position(left)"
        elif GPIO.input(left_sensor) == 0 and GPIO.input(right_sensor) == 1 and GPIO.input(center_sensor) == 0:
            line = "position(left)"
        elif GPIO.input(center_sensor) == 0 and GPIO.input(right_sensor) == 0 and GPIO.input(left_sensor) == 0:
            line = "position(across)"
        else:
            line = "position(lost)"
        sense = "{}, {}, {}".format(GPIO.input(left_sensor), GPIO.input(center_sensor), GPIO.input(right_sensor))
        rospy.loginfo(sense)
        pub.publish(line)
        rate.sleep()


if __name__ == '__main__':
    try:
        liner()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
