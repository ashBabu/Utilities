#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosbag
import sys

#  First the pointCloud2 is recorded from kinect camera and saved. Use this program to get a new filtered rosbag which
# played would not cause any time jumping error or message filter error

def pointCloudAsObstacle():
    with rosbag.Bag('/home/automato/Ash/scripts/ceiling_cam_qhd_points_filtered.bag', 'w') as write_bag:
        with rosbag.Bag('/home/automato/Ash/scripts/ceiling_cam_qhd_points.bag', 'r') as read_bag:
            for topic, pc_msg, t in read_bag.read_messages():
                pc_msg.header.stamp.secs = 0
                pc_msg.header.stamp.nsecs = 0
                write_bag.write(topic, pc_msg)

    sys.exit()


if __name__ == '__main__':

    pointCloudAsObstacle()
