import rospy
import math
from std_msgs.msg import Float32, Float32MultiArray ,Int32
from time import sleep

WHEEL_RADIUS = 0.0425
WHEEL_DISTANCE =0.141*2

Timer = 0 
def pi_2_pi(angle):
    # a function to Brap the angle between -pi and pi (for numerical stability)
    return (angle + math.pi) % (2 * math.pi) - math.pi

def forward_model(Bl, Br,Fl,Fr):
    # units are m/s and rad/s
    xdot = (Br * WHEEL_RADIUS + Bl * WHEEL_RADIUS+Fr * WHEEL_RADIUS + Fl * WHEEL_RADIUS) / 4 #Vrobot
    thetadot = ((Br+Fr) * WHEEL_RADIUS - (Bl+Fl) * WHEEL_RADIUS) /(2* WHEEL_DISTANCE) #theta dot of robot
    return xdot, thetadot

def BL_callback(msg):
    global Bl
    Bl = msg.data

def BR_callback(msg):
    global Br
    Br = msg.data
    
def FL_callback(msg):
    global Fl
    Fl = msg.data

def FR_callback(msg):
    global Fr
    Fr = msg.data

if __name__ == "__main__":

    # Initialize the ROS node
    rospy.init_node("localizer")

    # Initialize wheel speed values 
    Bl, Br , Fl, Fr = 0, 0, 0, 0

    # Publisher for the pose topic 
    pose_pub = rospy.Publisher("/localizer/pose", Float32MultiArray, queue_size= 10)

    # Subscriber to the velocity commanding topic
    rospy.Subscriber("/arduino/BL", Float32, BL_callback)
    rospy.Subscriber("/arduino/BR", Float32, BR_callback)
    rospy.Subscriber("/arduino/FL", Float32, FL_callback)
    rospy.Subscriber("/arduino/FR", Float32, FR_callback)

    # Initialize time for integration
    t_start = rospy.get_time()
    t_prev = rospy.get_time()

    # Initialize variables for trapezoidal integration
    xdot_prev, thetadot_prev = 0, 0

    # Initialize integration values 
    x, y, theta, = 0, 0, 0
    
    while not rospy.is_shutdown():
        xdot, thetadot = forward_model(Bl, Br,Fl,Fr)
	
        # calculate dt
        t_start = rospy.get_time()
        dt = t_start-t_prev

        # do the integration (trapezoidal)
        x += ((xdot + xdot_prev)/2) * math.cos(theta) * dt
        y += ((xdot + xdot_prev)/2) * math.sin(theta) * dt
        theta += ((thetadot + thetadot_prev)/3.75) * dt 
        theta = pi_2_pi(theta) # Brap-to-pi
        t_prev = t_start
        #rospy.loginfo(str(x)+","+str(y)+","+str(theta))
        # equate variables for next iteration
        xdot_prev = xdot
        thetadot_prev = thetadot

        # initialize ros message to be published
        msg = Float32MultiArray()
        msg.data.append(x)
        msg.data.append(y)
        msg.data.append(theta)
        pose_pub.publish(msg)
        
