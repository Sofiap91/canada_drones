#! /usr/bin/env python
import time
import rospy
from drone_msgs.srv import Commands, CommandsResponse
from std_msgs.msg import Empty


def callback(request):
    response = CommandsResponse()
    if request.command == "takeoff":
        takeoff_pub_.publish(empty_msg)
        response.success = True
        return response
    elif request.command == "land":
        land_pub_.publish(empty_msg)
        response.success = True
        return response


rospy.init_node("drone_service_srv")
drone_control_srv = rospy.Service('/drone_control_srv', Commands, callback)
takeoff_pub_ = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
land_pub_ = rospy.Publisher('/drone/land', Empty, queue_size=1)

empty_msg = Empty()

ctrl_c = False


def shutdownhook():
    global ctrl_c
    rospy.logdebug("Shutting down...")


rospy.on_shutdown(shutdownhook)

rospy.spin()
