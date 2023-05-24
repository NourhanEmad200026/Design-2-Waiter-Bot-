import rospy
import math
from std_msgs.msg import Float32, Float32MultiArray, Int32
import serial
import time
ACTUAL_RR = 0
ACTUAL_RL = 0
ACTUAL_FR = 0
ACTUAL_FL = 0
DISTANCE = 0
USR = 0
USL = 0
left = 0
right = 0
direction = 0
stop = 0
lift = 0

def left_clbk(msg):
    global left
    left = msg.data

def right_clbk(msg):
    global right
    right = msg.data

def direction_clbk(msg):
    global direction
    direction = msg.data

def stop_clbk(msg):
    global stop
    stop = msg.data
    
def lift_clbk(msg):
    global lift
    lift = msg.data
    

if __name__ == '__main__':
    rospy.init_node("arduino")
    BL_pub = rospy.Publisher("/arduino/BL", Float32, queue_size= 10)
    BR_pub = rospy.Publisher("/arduino/BR", Float32, queue_size= 10)
    FL_pub = rospy.Publisher("/arduino/FL", Float32, queue_size= 10)
    FR_pub = rospy.Publisher("/arduino/FR", Float32, queue_size= 10)
    
    USR_pub = rospy.Publisher("/arduino/Proximity_sensorR", Int32, queue_size= 10)
    USL_pub = rospy.Publisher("/arduino/Proximity_sensorL", Int32, queue_size= 10)
    
    Lift_pub = rospy.Publisher("/arduino/Lift", Float32, queue_size= 10)
    
    rospy.Subscriber("/navigator/left", Int32 , left_clbk)
    rospy.Subscriber("/navigator/right", Int32 , right_clbk)
    rospy.Subscriber("/navigator/direction", Int32 , direction_clbk)
    rospy.Subscriber("/navigator/stopper", Int32 , stop_clbk)
    rospy.Subscriber("/navigator/lift", Int32 , lift_clbk)
    
    ser = serial.Serial('/dev/ttyACM0', 19200, timeout=1)
    
     # Setup ROS rate 
    rate = rospy.Rate(10) # 10 Hz 
    
    ser.reset_input_buffer()
    while True:
        
        #  READING FROM ARDUINO
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            datalist = line.split(",")
            BR_pub.publish(int(datalist[0]))
            BL_pub.publish(int(datalist[1]))
            FR_pub.publish(int(datalist[2]))
            FL_pub.publish(int(datalist[3]))
            Lift_pub.publish(int(datalist[4]))
            USR_pub.publish(int(datalist[5]))
            USL_pub.publish(int(datalist[6]))
            
            
            
        ################################################
        #  WRITING TO ARDUINO
        number = 1*direction + 10* right + 100*left +1000*lift + 10000*stop
        ser.write(str(number).encode())
        ################################################
        
        rate.sleep()
