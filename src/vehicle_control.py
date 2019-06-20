#!/usr/bin/env python

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import math
import PIL
import ros_numpy


stop = False
stopMessage = AckermannDrive(steering_angle=0, steering_angle_velocity=0, speed=0, acceleration=0, jerk=0)

throttle_per = 0.7
distance_limit = 20, 2
width_limit = 3, 3


def interpretDanger(data):
    global stop
    data_np = ros_numpy.numpify(data)
    center = data_np.shape[0] // 2
    data_np = data_np[(center - distance_limit[0]):(center - distance_limit[1]), (center - width_limit[0]):(center + width_limit[1])]

    if data_np.max() > 100:
        stop = True
        rospy.loginfo('STOPPING')

def main():
    global stop
    '''
        Main Method to be run, allows handling of ROSInterrupt
    '''
    
    #init node
    rospy.init_node('vehicle_control', anonymous=True)
    
    #Vehicle control cmd publisher
    vehicle_cmd = rospy.Publisher('carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)
    
    #vehicle autonomous publisher
    vehicle_autonomous = rospy.Publisher('carla/ego_vehicle/enable_autopilot', Bool, queue_size=10)
    
    #Lidar proximity map Subscriber
    rospy.Subscriber('/top_down_view_mask', Image, interpretDanger)

    #refresh rate
    rate = rospy.Rate(10)
 
    while not rospy.is_shutdown():
        '''
            Main Loop
        '''

        #vehicle_cmd generation
        vehicle_cmd_val = AckermannDrive(steering_angle=0, steering_angle_velocity=0, speed=7, acceleration=0, jerk=0)
        #rospy.loginfo(vehicle_cmd_val)
        
        #vehicle autonomous generation
        vehicle_autonomous_val = False
        
        #publish values
        if not vehicle_autonomous_val:
            if not stop:
                vehicle_cmd.publish(vehicle_cmd_val)
            else:
                rospy.loginfo('SENDING STOP MESSAGE')
                vehicle_cmd.publish(stopMessage)

                rospy.sleep(3)

                vehicle_cmd.publish(AckermannDrive(steering_angle=1, steering_angle_velocity=0, speed=-20, acceleration=0, jerk=0))
                rospy.sleep(1)
                vehicle_cmd.publish(AckermannDrive(steering_angle=1, steering_angle_velocity=0, speed=-20, acceleration=0, jerk=0))
                rospy.sleep(1)
                vehicle_cmd.publish(AckermannDrive(steering_angle=1, steering_angle_velocity=0, speed=-20, acceleration=0, jerk=0))
                rospy.sleep(1)
                vehicle_cmd.publish(AckermannDrive(steering_angle=1, steering_angle_velocity=0, speed=-20, acceleration=0, jerk=0))
                rospy.sleep(1)
                

                vehicle_cmd.publish(stopMessage)
                rospy.sleep(1)
                vehicle_cmd.publish(stopMessage)
                rospy.sleep(1)
                

                stop = False
                vehicle_cmd.publish(vehicle_cmd_val)

        elif not stop:
            vehicle_autonomous.publish(vehicle_autonomous_val)
        else:
            vehicle_autonomous.publish(False)
            vehicle_cmd.publish(stopMessage)

        #idle for rate
        rate.sleep()
        


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass