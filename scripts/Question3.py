#! /usr/bin/env python
import rospy
import math
import time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from apriltag_ros.msg import *
from geometry_msgs.msg import TwistStamped

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

def land():
    print("Landing & Dis-Arming")
    result =  arming_client(value=False)

# Tag based landing 
def tag_land():
    while(not rospy.is_shutdown()):
        try:
            rospy.Subscriber("/tag_detections", AprilTagDetectionArray, pose_callback)
        except:
            pass

def pose_callback(msg):
    # Camera is rotated 90 degree with respect to body so x turns y and y turns x 
    x = msg.detections[0].pose.pose.pose.position.y 
    y = msg.detections[0].pose.pose.pose.position.x 
    z = msg.detections[0].pose.pose.pose.position.z
    
    #Calculating cmd velocity towards the tag using a Position to Velocity controller 
    Cmd_vel = PD.command(x,y,z)
    
    command = TwistStamped()
    command.twist = Twist()
    command.header = Header()
    command.header.stamp = rospy.Time.now()
    
    command.twist.linear.x = Cmd_vel[0]
    command.twist.linear.y = Cmd_vel[1]
    command.twist.linear.z = Cmd_vel[2]
    command.twist.angular.x = 0
    command.twist.angular.y = 0
    command.twist.angular.z = 0
    
    #Landing if very close to the groud and tag
    # if PD.ret_z() < 0.1 :
    #     print("Landing & Dis-Arming")
    #     result =  arming_client(value=False)
    # else:
    #     pass
    
    pub.publish(command)

#PD controller class for 
class controller:
    def __init__(self):
        self.x_er_prev = 0
        self.y_er_prev = 0
        self.z_er_prev = 0
        self.kp = 0.2
        self.kd = 0.2
        self.x_er = 0
        self.y_er = 0
        self.z_er = 0
        self.dt = 0.1
    
    def pos(self,x,y,z):
        self.x_er = x
        self.y_er = y
        self.z_er = z
    
    def ret_x(self):
        return self.x_er
    
    def ret_y(self):
        return self.y_er   
    
    def ret_z(self):
        return self.z_er     
    
    def command(self,x,y,z):
        self.x_er = x
        self.y_er = y
        self.z_er = z
        x_vel = self.x_er*self.kp + (self.x_er-self.x_er_prev)/self.dt
        y_vel = self.y_er*self.kp + (self.y_er-self.y_er_prev)/self.dt
        z_vel = self.z_er*self.kp + (self.z_er-self.z_er_prev)/self.dt
        self.x_er_prev = self.x_er
        self.y_er_prev = self.y_er
        self.z_er_prev = self.z_er
        print(x_vel, y_vel,z_vel)
        return [x_vel, y_vel,-z_vel]

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)  
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Setpoint publishing Rate
    rate = rospy.Rate(20)
    
    PD  = controller()
    # Setting the takeoff height
    pose = PoseStamped()
    pose.pose.position.x = 0.1
    pose.pose.position.y = 0.1
    pose.pose.position.z = 2

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

    # Landing using the PD controller and Tag 
    tag_land()
    


    



    