import rospy
import math
from time import sleep
from std_msgs.msg import Float32, Float32MultiArray, Int32


x,y,xchanged,ychanged = 0,0,0,0

def appx_clbk(msg):
    global x
    global xchanged
    x = msg.data
    xchanged = 1 

def appy_clbk(msg):
    global y
    global ychanged
    y = msg.data
    ychanged = 1 
    

if __name__ == "__main__":
    rospy.init_node("Controller_node")
    
    TARGETTING = rospy.Publisher("/controller/target", Float32MultiArray , queue_size=10)
    rospy.Subscriber("/app/x", Float32 , appx_clbk)
    rospy.Subscriber("/app/y", Float32 , appy_clbk)


    while True:
        if xchanged==1 and ychanged ==1 :
            msg = Float32MultiArray()
            msg.data.append(float(x))
            msg.data.append(float(y))
            TARGETTING.publish(msg)
            xchanged = 0
            ychanged = 0 
