#! /usr/bin/env python
import rospy
import numpy as np
from stl import mesh
from drone_msgs.srv import PosesList, PosesListResponse
from drone_msgs.msg import Poses

def callback(request):
    response = PosesListResponse()
    
    my_mesh = mesh.Mesh.from_file('/home/user/cube_98vert.stl')
    points = np.around(np.unique(my_mesh.vectors.reshape([my_mesh.vectors.size/3, 3]), axis=0), 2)

    list_points = points.tolist()
    print("Points are", list_points)
    list_i = Poses()

    for i in range(len(list_points)):
        list_i.poses = list_points[i]
        print(list_i)
        response.pose_list[i] = list_i.poses

    return response


rospy.init_node("positions_service_srv")
pos_srv = rospy.Service('/stl_positions_srv', PosesList, callback)

ctrl_c = False


def shutdownhook():
    global ctrl_c
    rospy.logdebug("Shutting down...")


rospy.on_shutdown(shutdownhook)

rospy.spin()