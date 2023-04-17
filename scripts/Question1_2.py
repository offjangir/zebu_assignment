#! /usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# Defining a arming function for arming the drone
def arm():
    print("Arming")
    result =  arming_client(value=True)
    print(result)

# Defining a offboard control function for the drone
def off_b():
    print("Setting Offboard Mode")
    result = set_mode_client(custom_mode="OFFBOARD")
    print(result)

# Takeoff 
def takeoff(takeoff_time):
    for i in range(takeoff_time):
        local_pos_pub.publish(pose)
        time.sleep(1)

# Return Home
def go_home():
    for i in range(5):
        local_pos_pub.publish(pose)
        time.sleep(1)
    
    for i in range(5):
        pose.pose.position.z = 0
        local_pos_pub.publish(pose)

        time.sleep(1)
    print("Landing & Dis-Arming")
    result =  arming_client(value=False)

# Spiral Trejectory Function
def spiral_trej():
    pose_spiral = PoseStamped()
    rate = rospy.Rate(10)
    # Spiral equation for position based spiral :
    #     x = r * sin(2*pi*n*t)
    #     x = Vf * t 
    #     z = r * cos(2*pi*n*t)
    #  Taking   r = 2 
                # Vf = 0.5  
                # t per step = 1/20 
                # n = 0.5
    t = 0
    while True:
        pose_spiral.pose.position.x = 2*(math.sin(math.pi*t))
        pose_spiral.pose.position.y = 0.5 * t
        pose_spiral.pose.position.z = pose.pose.position.z + 2*(math.cos(math.pi*t)) 
        # +pose.pose.position.z For Shifting the origin to takeoff height Avoid collision to ground
        t += 1/20
        local_pos_pub.publish(pose_spiral)
        
        # Returns Home after 10 Second
        if t >= 10:
            print("Returning Home")
            break
        rate.sleep()
                

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)  
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # Setpoint publishing Rate
    rate = rospy.Rate(20)

    # Setting the takeoff height
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 5

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()



    #Arming and Setting Offboard Ground Station Control
    arm() 
    off_b() 

    #Taking Off and Holding Position
    takeoff_time = 10
    takeoff(takeoff_time)

    #Spiral Trejectory
    spiral_trej()
    
    #Landing and Reaching Home Position
    go_home()



    