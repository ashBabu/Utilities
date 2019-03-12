#!/usr/bin/env python
import rospy

# from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
# from geometry_msgs.msg import PoseArray
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from people_msgs.msg import PositionMeasurement
from people_msgs.msg import PositionMeasurementArray
# from nav_msgs.msg import Odometry
# from requests import post
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA

import json

import tf
import numpy as np

import math
#import statistics

rospy.init_node('virtual_people_gt',disable_signals=True)

publisher = rospy.Publisher('/people_gt', PositionMeasurementArray, queue_size=10)
publisher_marker = rospy.Publisher('/visualization_gt', Marker, queue_size=10)
#publisher_marray = rospy.Publisher('/people_tracker/marker_array', MarkerArray, queue_size=10)
human_motion = rospy.get_param('~human_motion','LINEAR')
rate=rospy.Rate(50)
counter=0
people_inside=False

## Human Trajectory
a=0.01
b=0.2
c=0.5
d=0.5
t = np.arange(-15,15,0.01)
k=1

while k < t.size:
    j = PositionMeasurementArray()
    j.header = Header()
    #j.header.stamp = {'secs' : datetime.datetime.now().time().second , 'nsecs' : datetime.datetime.now().time().microsecond}
    now = rospy.Time.now()
    j.header.stamp.secs = now.secs
    j.header.stamp.nsecs = now.nsecs
    j.header.frame_id = '/base_link'

    pose1 = PositionMeasurement()
    if human_motion == 'LINEAR':
        pose1.pos.x = t[k]    
        pose1.pos.y = c * pose1.pos.x + d
    elif human_motion == 'NONLINEAR':
        pose1.pos.x = t[k]
        pose1.pos.y = a*(pose1.pos.x*pose1.pos.x*pose1.pos.x)+b*(pose1.pos.x*pose1.pos.x) + c*pose1.pos.x + d
    elif human_motion == 'FIGURE8':
        pose1.pos.x = 15 * np.cos(t[k]) + 10
        pose1.pos.y = 5 * np.sin(2*t[k]) + 10

    pose2 = PositionMeasurement()
    pose2.pos.x = 1
    pose2.pos.y = -1

    pose1_marker = Pose()
    pose1_marker.position = pose1.pos

    markerPeople = Marker()
    markerPeople.header = Header()
    markerPeople.header.stamp.secs = now.secs
    markerPeople.header.stamp.nsecs = now.nsecs
    markerPeople.header.frame_id = '/odom_combined'
    markerPeople.ns="PEOPLE"
    markerPeople.id=1
    markerPeople.type=markerPeople.SPHERE
    scalePeople = Vector3()
    scalePeople.x=0.2
    scalePeople.y=0.2
    scalePeople.z=0.2
    markerPeople.scale=scalePeople
    colorPeople = ColorRGBA()
    colorPeople.r=0.0
    colorPeople.g=0.0
    colorPeople.b=1.0
    colorPeople.a=1.0
    markerPeople.color = colorPeople
    markerPeople.lifetime.secs=1
    markerPeople.lifetime.nsecs=0

    #markerArrayPeople = MarkerArray()


#    if (counter % 500)==0:
#        if people_inside:
#            people_inside=False
#        else:
    people_inside=True

    if people_inside:
        j.people.append(pose1)
 #       j.people.append(pose2)
        markerPeople.pose=pose1_marker
        #markerArrayPeople.markers.append(markerPeople)
    #print(j.header.seq)

    publisher.publish(j)
    publisher_marker.publish(markerPeople)
    #publisher_marray.publish(markerArrayPeople)
    counter = counter + 1
    rate.sleep()
    k = k + 1

if k==t.size:
    rospy.signal_shutdown('Simulation is ending...')
else:
    rospy.spin()
