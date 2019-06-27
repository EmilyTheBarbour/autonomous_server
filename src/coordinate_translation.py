#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
import carla

carla_map = None
pubPath = rospy.Publisher('/carla/ego_vehicle/waypoints_translated', Path, queue_size=3)
pubPose = rospy.Publisher('/carla/ego_vehicle/pose_translated', Pose, queue_size=3)r

def convert_path(path):
    new_poses = []

    if isinstance(path, Path):
        for p in path.poses:
            transform = carla_map.transform_to_geolocation(carla.Location(
                x=p.pose.position.x,
                y=p.pose.position.y,
                z=p.pose.position.z
            ))

            new_poses.append(PoseStamped(
                header=p.header, 
                pose=Pose(
                    position=Point(
                        x=transform.latitude,
                        y=transform.longitude, 
                        z=transform.altitude
                    ),
                    orientation=p.pose.orientation
                )
            ))
    
        rospy.loginfo('converted {} points in waypoint path.'.format(len(new_poses)))
        pubPath.publish(Path(header=path.header, poses=new_poses))

def convert_pose(nav_sat):


def main():
    global carla_map
    carla_map = carla.Client('localhost', 2000).get_world().get_map()

    rospy.init_node('coordinate_translation', anonymous=True)
    rospy.Subscriber('carla/ego_vehicle/waypoints', Path, callback=convert_path)
    rospy.Subscriber('carla/ego_vehicle/waypoints', Path, callback=convert_path)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass