#!/usr/bin/env python

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import math
import PIL
import ros_numpy
import tf

waypoints = None
first_waypoints = True
center = None
goal_point = None
previous_node = None


stop = False
stopMessage = AckermannDrive(steering_angle=0, steering_angle_velocity=0, speed=0, acceleration=0, jerk=0)

distance_limit = 20, 2
width_limit = 3, 3
look_ahead = 5


def interpretDanger(data):
    global stop
    data_np = ros_numpy.numpify(data)
    center = data_np.shape[0] // 2
    data_np = data_np[(center - distance_limit[0]):(center - distance_limit[1]), (center - width_limit[0]):(center + width_limit[1])]

    if data_np.max() > 100 and not stop:
        stop = True
        rospy.loginfo('STOPPING')

def convert_pose(odom):
    global center
    

    if isinstance(odom, Odometry):
        explicit_quat = [
            odom.pose.pose.orientation.x, 
            odom.pose.pose.orientation.y, 
            odom.pose.pose.orientation.z, 
            odom.pose.pose.orientation.w
        ]

        center = (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            np.rad2deg(tf.transformations.euler_from_quaternion(explicit_quat)[2] + 0 * np.pi * 1 / 6) 
        )

def convert_path(path):
    global waypoints, first_waypoints

    if first_waypoints:
        first_waypoints = False
        return
    
    if isinstance(path, Path):
        waypoints = np.zeros((len(path.poses), 2))

        for i, p in enumerate(path.poses):
            waypoints[i] = (p.pose.position.x, p.pose.position.y)
    

def find_intersections_along_path(waypoints, circle_center, circle_radius):
    '''
        Return a list of intersections and their ID (gained from the left parent)

        waypoints: np array with shape n * 2 , where n is the number of waypoints
        circle_center: np array with shape 2, x and y coordinates of the circle center
        circle_radius: scalar value demonstrating the radius of the circle

        returns:

            intersections: np array with shape n * 3, where n is the number of intersections
                            along the circle with the given path, and 3 is an ordered tuple where
                            each index returns a specific value:
                                0: ID of the intersection
                                1: x of the intersection
                                2: y of the intersection
        
        webpage http://mathworld.wolfram.com/Circle-LineIntersection.html was utilized for the 
        intersection math
    '''

    #translate circle center to origin
    intersections = np.zeros((0, 3)) 
    waypoints[:,0] -= circle_center[0]
    waypoints[:,1] -= circle_center[1]

    for i in range(len(waypoints) - 1):
        dx = np.subtract(waypoints[i+1, 0], waypoints[i,0])
        dy = np.subtract(waypoints[i+1, 1], waypoints[i, 1])
        dr = np.sqrt(np.add(dx**2, dy**2))
        D = np.subtract(np.multiply(waypoints[i, 0], waypoints[i+1, 1]), np.multiply(waypoints[i + 1, 0], waypoints[i, 1]))

        delta = circle_radius**2 * dr**2 - D**2

        x1 = np.divide(np.add(np.multiply(D, dy), np.multiply(np.sign(dy), np.multiply(dx, np.sqrt(delta)))),dr**2)
        x2 = np.divide(np.subtract(np.multiply(D, dy), np.multiply(np.sign(dy), np.multiply(dx, np.sqrt(delta)))),dr**2)
        y1 = np.divide(np.add(np.multiply(-D, dx), np.multiply(np.abs(dy), np.sqrt(delta))),dr**2)
        y2 = np.divide(np.subtract(np.multiply(-D, dx), np.multiply(np.abs(dy), np.sqrt(delta))),dr**2)

        if not np.isnan(x1) and (x1 >= waypoints[i:i+2,0]).any() and (x1 <= waypoints[i:i+2,0]).any():
            intersections = np.vstack((intersections, np.array([i, x1,y1])))
        if not np.isnan(x2) and (x2 >= waypoints[i:i+2,0]).any() and (x2 <= waypoints[i:i+2,0]).any():
            intersections = np.vstack((intersections, np.array([i, x2,y2])))
    

    # retranslate points back to original position
    waypoints[:,0] += circle_center[0]
    waypoints[:,1] += circle_center[1]
    intersections[:,1] += circle_center[0]
    intersections[:,2] += circle_center[1]

    return intersections

def find_difference(target, source):
    a = target - source

    if a > 180:
        a -= 360
    elif a < -180:
        a+= 360

    return a
    

def main():
    global stop, center, waypoints
    '''
        Main Method to be run, allows handling of ROSInterrupt
    '''
    
    #init node
    rospy.init_node('vehicle_control', anonymous=True)
    
    #Vehicle control cmd publisher
    vehicle_cmd = rospy.Publisher('carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)
    target_pub = rospy.Publisher('target_point', PointStamped, queue_size=1)


    #Lidar proximity map Subscriber
    rospy.Subscriber('/top_down_view_mask', Image, interpretDanger)

    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, callback=convert_pose)
    rospy.Subscriber('carla/ego_vehicle/waypoints', Path, callback=convert_path)

    #refresh rate
    rate = rospy.Rate(10)
 
    while not rospy.is_shutdown():
        '''
            Main Loop
        '''

        if center is not None and waypoints is not None:

            heading = center[2]

            inter = []
            i = 0

            while len(inter) == 0:
                inter = find_intersections_along_path(waypoints, center, look_ahead + i)
                i += 0.5

            angles = np.zeros((len(inter)))
            closest_angle, angle_id = 360, -1

            for i, point in enumerate(inter):
                angles[i] = np.rad2deg(np.arctan2(point[1] - center[1], point[0] - center[0]))
                dif = find_difference(angles[i], heading)
        
                if abs(dif) < closest_angle:
                    closest_angle, angle_id = abs(dif), i


            target_angle = find_difference(angles[angle_id], heading)
            rospy.loginfo('\nHEADING: {}\nCHANGE: {}'.format(heading, target_angle))
            
            target_pub.publish(PointStamped(point=Point(x=inter[angle_id][0], y=inter[angle_id][1], z=inter[angle_id][2])))

            vehicle_cmd.publish(AckermannDrive(steering_angle=np.deg2rad(target_angle), steering_angle_velocity=0, speed=5, acceleration=0, jerk=0))    


        else:
            vehicle_cmd.publish(stopMessage)

        #idle for rate
        rate.sleep()
        


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass