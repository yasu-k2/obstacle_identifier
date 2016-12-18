#!/usr/bin/env python

import rospy, math, random
import numpy as np
from obstacle_identifier.srv import IdentifyObstacle, IdentifyObstacleResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def point_cloud_callback(point_cloud):
    global left, right
    # With respect to /camera
    # define right obstacle area in [m]
    xra = 0.05
    xrb = 0.55
    zra = 0.95
    zrb = 1.25
    # define left obstacle area in [m]
    xla = -0.55
    xlb = -0.05
    zla = 1.25
    zlb = 1.55
    
    z_left = np.array([])
    z_right = np.array([])
    for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
        # Printing point cloud data
        #print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
        # xmin : -1.9  xmax: 1.7 right
        # ymin : -1.9  ymax: 0.7 down
        # zmin : 0     zmax: 5.4 forward
        if (p[0]>=xla and p[0]<=xlb and p[2]>=zla and p[2]<=zlb):
            z_left = np.append(z_left, p[1])
            #print "z_left %f"%p[1]
        elif (p[0]>=xra and p[0]<=xrb and p[2]>=zra and p[2]<=zrb):
            z_right = np.append(z_right, p[1])
            #print "z_right %f"%p[1]
    if len(z_left) > 0 and len(z_right) > 0:
        left = np.average(z_left)
        right = np.average(z_right)
    else:
        if len(z_left) == 0:
            right = 0
            left = 1
        else:
            left = 0
            right = 1
    #print " left: %f right: %f" %(left,right)

def handle_obstacle_identification(req):
    global left, right
    if req.mode == 1:
        # Obstacle on the smaller side
        #print " left: %f right: %f" %(left,right)
        case = 1 if (right <= left) else 2
    else:
        case = 0
    return IdentifyObstacleResponse(case)

def obstacle_identification_server():
    global left, right
    left = 0
    right = 0
    rospy.init_node('obstacle_identification_server', anonymous=True)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_cloud_callback)
    ser = rospy.Service('obstacle_identification', IdentifyObstacle, handle_obstacle_identification)
    rospy.spin()

if __name__=='__main__':
    obstacle_identification_server()
     
"""
VERSION 1

import sensor_msgs.point_cloud2 as pc2

for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
     print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])


VERSION 2 (alternative)

import sensor_msgs.point_cloud2
...
for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]

SET UP FOR RVIZ
    rosrun tf static_transform_publisher 0 0 0 0 0 0 /camera /camera_rgb_optical_frame 10

    -listen on /camera/depth_registered/points
    -add /robot_base to /camera_rgb_optical_frame transform (need to fix transforms) [??]

    -msg type: sensor_msgs/PointCloud2
            std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        uint32 height`
        uint32 width
        sensor_msgs/PointField[] fields
          uint8 INT8=1
          uint8 UINT8=2
          uint8 INT16=3
          uint8 UINT16=4
          uint8 INT32=5
          uint8 UINT32=6
          uint8 FLOAT32=7
          uint8 FLOAT64=8
          string name
          uint32 offset
          uint8 datatype
          uint32 count
        bool is_bigendian
        uint32 point_step
        uint32 row_step
        uint8[] data
        bool is_dense
"""
