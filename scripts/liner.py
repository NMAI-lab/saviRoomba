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


def get_line():
    centerLED = GPIO.input(center_sensor)
    leftLED = GPIO.input(left_sensor)
    rightLED = GPIO.input(right_sensor)
    LEDs = (leftLED, centerLED, rightLED)
    
    # 0 indicates the location of the line
    if LEDs == (1,0,1):
        result = "c"
    elif (LEDs == (1,1,0)) or (LEDs == (1,0,0)):
        result = "r"
    elif (LEDs == (0,1,1)) or (LEDs == (0,0,1)):
        result = "l"
    elif LEDs == (0,0,0):
        result = "a"
    else:
        result = "ll"
        
    return (result, LEDs)
    

def liner():
    pub = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.init_node('liner', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        (linePosition, LEDs) = get_line()        

        if linePosition == "c":
            line = "line(center)"
        elif linePosition == "r":
            line = "line(right)"
        elif linePosition == "l":
            line = "line(left)"
        elif linePosition == "a":
            line = "line(across)"
        else:
            line = "line(lost)"
        sense = "{}, {}, {}".format(LEDs[0], LEDs[1], LEDs[2])
        rospy.loginfo(sense)
        pub.publish(line)
        rate.sleep()


if __name__ == '__main__':
    try:
        liner()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
