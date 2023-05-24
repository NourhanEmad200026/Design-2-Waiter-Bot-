import rospy
import math
from time import sleep
from std_msgs.msg import Float32, Float32MultiArray, Int32
from sys import setrecursionlimit

setrecursionlimit(10000)

WHEEL_RADIUS = 0.0425
WHEEL_DISTANCE = 0.141*2

MAX = 0.1
MIN = 0.0
x=0.0
y=0.0
theta = 0.0
obav = 0.5

Timer = 0 

YAW_THRESHOLD = 15.0*math.pi/180 
DIST_THRESHOLD = 0.1

TARGET = [3.0,3.0]
HOME = [0.0,0.0]
proximity_right = 0
proximity_left = 0


def prx1_clbk(msg):
    global proximity_right
    proximity_right = msg.data

def prx2_clbk(msg):
    global proximity_left
    proximity_left = msg.data # ba5od el values mn vrep


def pose_clbk(msg):
    global x, y, theta
    x = msg.data[0]
    y = msg.data[1]
    theta = msg.data[2] # ba5od el values mel odometry


def euclidean_distance(x_, y_, goal_point_):
    dist_x = goal_point_[0] - x_
    dist_y = goal_point_[1] - y_
    dist1 = float(dist_x*dist_x + dist_y*dist_y)
    dist = math.sqrt(dist1)
    return dist

def pi_2_pi(angle):
    # a function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + math.pi) % (2 * math.pi) - math.pi

def steering_angle(x_, y_, theta_, goal_point_):
    deltax = goal_point_[0] - x_
    deltay = goal_point_[1] - y_
    if   deltax >= 0 and deltay >= 0:
        #first quarter
        angle = math.atan2(abs(deltay),abs(deltax))
    elif deltax <= 0 and deltay >= 0:
        #second quarter
        angle = math.pi - math.atan2(abs(deltay),abs(deltax))
    elif deltax <= 0 and deltay <= 0:
        #third quarter
        angle = math.pi + math.atan2(abs(deltay),abs(deltax))
    elif deltax >= 0 and deltay <= 0: 
        #fourth quarter
        angle = -1.0 * math.atan2(abs(deltay),abs(deltax))
        
    return pi_2_pi(angle-theta_)


def moveme(R,L,F):
    left_wheel.publish(L)
    right_wheel.publish(R)
    forward.publish(F)
     
def GoToGOAL(goal_point):
    while euclidean_distance(x, y, goal_point) > DIST_THRESHOLD and not rospy.is_shutdown():
        if  proximity_right and  proximity_left:
                moveme(0,0,0)
                mysleep(0.5)
                moveme(0,1,1)
                mysleep(1)
                moveme(0,0,1)
                mysleep(1)
                if euclidean_distance(x, y, goal_point) < 0.5 :
                    break
        elif proximity_right:
            moveme(1,0,1)
            if euclidean_distance(x, y, goal_point) < 0.5 :
                break
        elif proximity_left:
            moveme(0,1,1)
            if euclidean_distance(x, y, goal_point) < 0.5 :
                break
        else:
            Temporaryvar = 1.0 * steering_angle(x,y,theta,goal_point)
            if abs(Temporaryvar) > YAW_THRESHOLD :
                if Temporaryvar > 0:
                    moveme(1,0,1)
                else:
                    moveme(0,1,1)
            else:
                moveme(0,0,1) # Safe to navigate
    goalreached.publish(1)


def GoToHome():
    TARGET = [0,0]
    GoToGOAL(HOME)
    while abs(theta) > YAW_THRESHOLD:
        if theta > 0:
            moveme(0,1,1)
        else:
            moveme(1,0,1)
    
def LIFT():
    Upper_pub.publish(1)

def LOWER():
    Upper_pub.publish(0)

def mainsequence():
    stopper.publish(0)
    GoToGOAL(TARGET)
    stopper.publish(1)
    LIFT()
    #rospy.loginfo("Robot Reached Goal @"+str(TARGET[0])+","+str(TARGET[1]))
    sleep(15)
    LOWER()
    mysleep(5)
    stopper.publish(0)
    GoToHome()
    stopper.publish(1)

                
    rospy.loginfo("Robot Reached HOME")

    
def TARGETTING(msg):
    # Initialize pose values
    x, y, theta = 0, 0, 0
    TARGET[0] = msg.data[0]
    TARGET[1] = msg.data[1]
    #rospy.loginfo("Target Changed")
    mainsequence()
    
if __name__ == "__main__":

    rospy.init_node("navigator_node")

    # Subscriber to the velocity commanding topic
    rospy.Subscriber("/localizer/pose", Float32MultiArray , pose_clbk)
    
    rospy.Subscriber("/controller/target", Float32MultiArray , TARGETTING)
    
    rospy.Subscriber("/arduino/Proximity_sensorR", Int32,prx1_clbk)
    rospy.Subscriber("/arduino/Proximity_sensorL", Int32,prx2_clbk)
    
    # Setup direction publishers
    left_wheel = rospy.Publisher("/navigator/left", Int32, queue_size=10)
    right_wheel = rospy.Publisher("/navigator/right", Int32, queue_size=10)
    forward = rospy.Publisher("/navigator/direction", Int32, queue_size=10)
    stopper = rospy.Publisher("/navigator/stopper", Int32, queue_size=10)
    #setup Lifting
    Upper_pub = rospy.Publisher("/navigator/lift", Int32, queue_size=10)
    goalreached =  rospy.Publisher("/navigator/goal", Int32, queue_size=10)
    LOWER()
    
    # Setup ROS rate 
    rate = rospy.Rate(10) # 10 Hz 

    #rospy.loginfo("Navigation node successfully spawned")
    #rospy.loginfo("Starting to traverse the path")
    while not rospy.is_shutdown():
        None

        
