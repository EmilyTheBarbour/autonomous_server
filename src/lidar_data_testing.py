#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField, Image
import math
from PIL import Image as pil_img

pubFull = rospy.Publisher('top_down_view', Image, queue_size=3)
pubMask = rospy.Publisher('top_down_view_mask', Image, queue_size=3)
img_size = 301
mask_size = 50
gain = 3.0


def callback(data):
    data_np = ros_numpy.numpify(data)

    f = open("output.txt", 'w')

    coords = np.zeros((2, data_np.shape[0]))
    coords[0] = data_np['x']
    coords[1] = data_np['y']

    z = data_np['z']


    coords = np.floor(np.multiply(np.divide(np.add(coords, 100), 200), img_size - 1)).astype(int)

    img_np = np.zeros((img_size, img_size, 3), dtype=np.uint8) 

    rospy.loginfo(img_np[2,2])
    rospy.loginfo(np.full((3), 5))

    for i in range(data_np.shape[0]):
        img_np[coords[0][i], coords[1][i]] += (z[i] * gain).astype(np.uint8)

    top_left_bounds = np.ceil(img_size / 2).astype(int) - mask_size // 2 
    bottom_right_bounds = np.ceil(img_size / 2).astype(int) + mask_size // 2 

    img_mask = np.flip(img_np[top_left_bounds:bottom_right_bounds, top_left_bounds:bottom_right_bounds])

    img_mask[25, 25] = (255, 0, 0)
    img_mask[24, 25] = (0, 255, 0)     
    img_mask[23, 25] = (0, 255, 0) 
    
    pubFull.publish(ros_numpy.msgify(Image, img_np, encoding='rgb8'))
    pubMask.publish(ros_numpy.msgify(Image, img_mask, encoding='rgb8'))


def main():
    rospy.init_node('lidar_data_testing', anonymous=True)
    rospy.Subscriber("/filtered_output", PointCloud2, callback=callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass