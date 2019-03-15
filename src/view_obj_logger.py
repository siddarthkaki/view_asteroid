#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import os
import sys
import getopt
import cv2 # OpenCV
import csv # CSV file tools

###########################

# Instantiate CvBridge
bridge = CvBridge()

# Command line argument input
# arg_pose_path = ''
# arg_img_path = ''
# if len(sys.argv) == 3:
#     arg_pose_path = str(sys.argv[2])
#     arg_img_path = str(sys.argv[3])

# Init csv writer
pose_path = os.path.expanduser('~/workspace/research/cygnus_data/pose')
#pose_path = os.path.expanduser(arg_pose_path)
pose_writer = csv.writer(open(os.path.join(pose_path,'pose_log.csv'), mode='w'), delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

# Callback function
def callback(odometry_msg, image_msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print odometry_msg.pose.pose

    secs = odometry_msg.header.stamp.secs
    nsecs = odometry_msg.header.stamp.nsecs

    pos_x = odometry_msg.pose.pose.position.x
    pos_y = odometry_msg.pose.pose.position.y
    pos_z = odometry_msg.pose.pose.position.z

    quat_x = odometry_msg.pose.pose.orientation.x
    quat_y = odometry_msg.pose.pose.orientation.y
    quat_z = odometry_msg.pose.pose.orientation.z
    quat_w = odometry_msg.pose.pose.orientation.w

    # Write pose to csv file
    pose_writer.writerow([secs, nsecs, pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w])

    # Save image to file
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        img_path = os.path.expanduser('~/workspace/research/cygnus_data/images')
        #img_path = os.path.expanduser(arg_img_path)
        img_name = str(secs) + '_' + str(nsecs) + '.jpeg'
        cv2.imwrite(os.path.join(img_path, img_name), cv2_img)
    
def logger():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('view_obj_logger', anonymous=True)

    # rospy.Subscriber("/view_obj/obj_pose", Odometry, callback)

    # Set-up message_filters subscribers for object odmetry and camera image
    odometry_sub = message_filters.Subscriber('/view_obj/obj_pose', Odometry)
    image_sub = message_filters.Subscriber('/camera/right/image_raw', Image)

    # Synchronise object pose and camera image
    tsync = message_filters.ApproximateTimeSynchronizer([odometry_sub, image_sub], queue_size=5, slop=0.1)
    tsync.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    logger()