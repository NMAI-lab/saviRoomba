#!/usr/bin/env python

import rospy
import socket
import threading
from std_msgs.msg import String

message = "post1,post1"

def remote_server():
    global message
    message = "post1,post1"

    serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serv.bind(('192.168.1.146', 12345))
    serv.listen(5)
    while True:
        conn, addr = serv.accept()

        try:
            print("connected to: {}".format(addr))

            while True:
                data = conn.recv(1024)
                if not data:
                    break
                message = str(data).rstrip()

                conn.send(bytes("Data received \n"))
        finally:
            conn.close()
            print('client disconnected')


def parse_msg():
    pub = rospy.Publisher('perceptions', String, queue_size=10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        params = message.split(",")

        if len(params) < 2:
            continue

        point = params[1].split("post")
        dest = "destination({})".format(point[1])

        rospy.loginfo(dest)
        pub.publish(dest)
        rate.sleep()


def app_receiver():
    rospy.init_node('appReceiver', anonymous=True)

    serverThread = threading.Thread(target=remote_server)
    serverThread.start()

    parserThread = threading.Thread(target=parse_msg)
    parserThread.start()


if __name__ == '__main__':
    try:
        app_receiver()
    except rospy.ROSInterruptException:
        pass
