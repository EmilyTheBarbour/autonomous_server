#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
import carla

current_pose = None
current_nav = None

def grab_current_pose(data):
    global current_pose

    if isinstance(data, Odometry):
        current_pose = data.pose
    else:
        rospy.loginfo("ERROR: ODEMETRY TOPIC NOT OF TYPE nav_msgs/Odometry")

def grab_current_nav(data):
    global current_nav

    if isinstance(data, NavSatFix):
        current_nav = data
    else:
        rospy.loginfo("ERROR: NAV TOPIC NOT OF TYPE sensor_msgs/NavSatFix")

def main():
    global current_nav, current_pose

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    rospy.init_node('gnss_vs_pose', anonymous=True)
    rospy.Subscriber('/carla/ego_vehicle/gnss/gnss1/fix', NavSatFix, callback=grab_current_nav)
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, callback=grab_current_pose)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if current_nav is not None and current_pose is not None:

            transform = carla_map.transform_to_geolocation(carla.Location(
                x=current_pose.pose.position.x,
                y=current_pose.pose.position.y,
                z=current_pose.pose.position.z,
                ))


            s = "GNSS:\n\tLAT: {}\n\tLON: {}\n\tALT: {}\nPOSE:\n\tLAT: {}\n\tLON: {}\n\tALT: {}".format(
                current_nav.latitude,
                current_nav.longitude,
                current_nav.altitude,
                transform.latitude,
                transform.longitude,
                transform.altitude
            )
        
            rospy.loginfo(s)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass