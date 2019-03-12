#!/usr/bin/env python

import rospy
# from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from people_msgs.msg import PositionMeasurement
from people_msgs.msg import PositionMeasurementArray
# from nav_msgs.msg import Odometry
# from requests import post
from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA

# import json
import sys
# import tf
import numpy as np

import math
# import statistics

class people_detector:

    def __init__(self):

        self.publisher = rospy.Publisher('/people_tracker_measurements', PoseArray, queue_size=10)
        self.publisher_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.publisher_gt = rospy.Publisher('/people_correct_position', PoseArray, queue_size=10)
        #publisher_marray = rospy.Publisher('/people_tracker/marker_array', MarkerArray, queue_size=10)

        self.s = rospy.Subscriber('/people_gt', PositionMeasurementArray, self.callback)

        self.fps = rospy.get_param('~sensor_fps',25)
        self.rate=rospy.Rate(self.fps)
        # counter=0

        ## Sensor Model
        self.sensor_angle_res = rospy.get_param('~sensor_bearing_res',0.1)
        self.sensor_distance_res = rospy.get_param('~sensor_range_res',0.5)        
        self.sensor_angle = np.arange(-math.pi,math.pi,self.sensor_angle_res)
        self.max_distance = rospy.get_param('~sensor_range_max',50)
        self.sensor_distance = np.arange(0,self.max_distance,self.sensor_distance_res)

        [self.sensor_mesh_angle,self.sensor_mesh_distance] = np.meshgrid(self.sensor_angle,self.sensor_distance)
        self.sensor_mesh_angle = self.sensor_mesh_angle.reshape(1,self.sensor_mesh_angle.size)
        self.sensor_mesh_distance = self.sensor_mesh_distance.reshape(1,self.sensor_mesh_distance.size)

        self.sensor_x = np.multiply(self.sensor_mesh_distance,np.cos(self.sensor_mesh_angle))
        self.sensor_y = np.multiply(self.sensor_mesh_distance,np.sin(self.sensor_mesh_angle))

        ## Data Acquisition
        self.humanCircleRadius = 0.2
        self.sensor_distance_noise_std = rospy.get_param('~sensor_range_noise_std',0.1)
        self.sensor_angle_noise_std = rospy.get_param('~sensor_bearing_noise_std',0.01)

    def cart2pol(self,x,y):
        r = np.sqrt(x ** 2 + y ** 2)
        t = np.arctan2(y, x)
        return (r,t)

    def pol2cart(self,r,t):
        x = r * np.cos(t)
        y = r * np.sin(t)
        return (x,y)

    def dataAcquisition(self,x,y):
        # print (rospy.Time.now())
        # print((x,y))
        # print((self.sensor_x.shape,self.sensor_y.shape))
        # Min.Polar distance
        # traj_r = repmat(r, numel(sensor_mesh_distance), 1);
        # traj_theta = repmat(theta, numel(sensor_mesh_angle), 1);

        # dist_polar = (sensor_mesh_distance(:).^ 2 + traj_r. ^ 2 - 2. * sensor_mesh_distance(:).*traj_r. * cos(sensor_mesh_angle(:) - traj_theta ));
        # [minDist, ind] = min(dist_polar);

        # [traj_x, traj_y] = pol2cart(traj_theta, traj_r);

        # dist_eucl = np.zeros([1,self.sensor_x.size])
        # for i in range(0,self.sensor_x.size):
        #     dist_eucl[0][i] = np.sqrt((x - self.sensor_x[0][i])**2 + (y - self.sensor_y[0][i])**2)
        dist_eucl = np.sqrt((np.subtract(x,self.sensor_x))**2 + (np.subtract(y,self.sensor_y))**2)

        # print (rospy.Time.now())
        minDist = np.min(dist_eucl)
        ind = np.argmin(dist_eucl)
        # print (minDist)
        if minDist <= self.humanCircleRadius:
            data_r = self.sensor_mesh_distance[0,ind]
            data_theta = self.sensor_mesh_angle[0,ind]
            data_x = self.sensor_x[0,ind]
            data_y = self.sensor_y[0,ind]

            obsv_r = data_r + np.random.randn() * self.sensor_distance_noise_std
            obsv_theta = data_theta + np.random.randn() * self.sensor_angle_noise_std

            [obsv_x, obsv_y] = self.pol2cart(obsv_r, obsv_theta)

            return (obsv_x,obsv_y)
        else:
            return (None,None)


    def callback(self,msg):
        j = PoseArray()
        j.header = msg.header
        now = rospy.Time.now()
        j.header.stamp.secs = now.secs
        j.header.stamp.nsecs = now.nsecs
        m = PoseArray()
        m.header = msg.header
        m.header.stamp.secs = now.secs
        m.header.stamp.nsecs = now.nsecs
        
        for det in range(0,len(msg.people)):
            pose = msg.people[det]
            x = pose.pos.x
            y = pose.pos.y
            # print (now)
            [obsv_x,obsv_y] = self.dataAcquisition(x,y)

        if obsv_y is not None:

            pose1 = Pose()
            # pose1.pos.x = now.nsecs % 10
            # pose1.pos.y = 3
            pose1.position.x = obsv_x
            pose1.position.y = obsv_y
            pose1.orientation.w = 1

            pose2 = Pose()
            pose2.position.x = x
            pose2.position.y = y
            pose2.orientation.w = 1

            pose1_marker = Pose()
            pose1_marker.position = pose1.position

            markerPeople = Marker()
            markerPeople.header = msg.header
            markerPeople.header.stamp.secs = now.secs
            markerPeople.header.stamp.nsecs = now.nsecs
            # markerPeople.header.frame_id = '/odom_combined'
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
            colorPeople.g=1.0
            colorPeople.b=0.0
            colorPeople.a=1.0
            markerPeople.color = colorPeople
            markerPeople.lifetime.secs=1
            markerPeople.lifetime.nsecs=0

            #markerArrayPeople = MarkerArray()


        #    if (counter % 500)==0:
        #        if people_inside:
        #            people_inside=False
        #        else:
        #     self.people_inside=True

            j.poses.append(pose1)
            m.poses.append(pose2)
            markerPeople.pose=pose1_marker
                #markerArrayPeople.markers.append(markerPeople)
            #print(j.header.seq)

            self.publisher.publish(j)
            self.publisher_marker.publish(markerPeople)
            self.publisher_gt.publish(m)
            #publisher_marray.publish(markerArrayPeople)
            # counter = counter + 1
            self.rate.sleep()
            # k = k + 1




def main(args):
    rospy.init_node('virtual_people_detector', anonymous = True)
    pd = people_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down'


if __name__ == '__main__':
    main(sys.argv)
