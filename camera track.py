import time
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int32


shouldread = 0
def make_chunks(EdgeArray,size_of_chunk):
    Chunks=[]
    for i in range(0,len(EdgeArray),size_of_chunk):
        Chunks.append(EdgeArray[i:i+size_of_chunk])
    return Chunks

def navigatorclbk(msg):
    global shouldread
    shouldread = 1


if __name__ == "__main__":
    cap=cv2.VideoCapture(0)
    
    lower_red = np.array([126, 90, 45], dtype = "uint8") 
    upper_red= np.array([179, 255, 255], dtype = "uint8")

    lower_green = np.array([66, 190, 77], dtype = "uint8") 
    upper_green= np.array([79, 255, 135], dtype = "uint8")

    lower_blue = np.array([87, 170,30], dtype = "uint8") 
    upper_blue= np.array([122, 255, 255], dtype = "uint8")



    rospy.init_node("camera_node")

    # Subscriber to the velocity commanding topic
    rospy.Subscriber("/navigator/goal", Int32 , navigatorclbk)
    
    RED = rospy.Publisher("/camera/red", Int32, queue_size=10)
    GRE = rospy.Publisher("/camera/green", Int32, queue_size=10)
    BLU = rospy.Publisher("/camera/blue", Int32, queue_size=10)
    
    USR_pub = rospy.Publisher("/arduino/Proximity_sensorR", Int32, queue_size= 10)
    USL_pub = rospy.Publisher("/arduino/Proximity_sensorL", Int32, queue_size= 10)
    
    

    while True:
        sleep(0.1)
        ret,frame=cap.read()
        
        if (shouldread == 1):
            frame=cv2.resize(frame,(640,480))
            red,green,blue = 0,0,0
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            
            mask = cv2.inRange(hsv, lower_red, upper_red)#kanet hsv
            _,mask=cv2.threshold(mask,0,255,cv2.THRESH_BINARY) 
            cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            for c in cnts:
                x=600
                if cv2.contourArea(c)>x:
                    x,y,w,h=cv2.boundingRect(c)
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
                    cv2.putText(frame,("DETECT"),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
                    red = 1
                    
                    
                    
            mask1 = cv2.inRange(hsv, lower_green, upper_green)#kanet hsv
            _,mask1=cv2.threshold(mask1,0,255,cv2.THRESH_BINARY) 
            cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            for c in cnts:
                x=600
                if cv2.contourArea(c)>x:
                    x,y,w,h=cv2.boundingRect(c)
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(frame,("DETECT"),(10,120),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
                    green = 1 
                    
                    
                    
            mask2 = cv2.inRange(hsv, lower_blue, upper_blue)#kanet hsv
            _,mask2=cv2.threshold(mask2,0,255,cv2.THRESH_BINARY) 
            cnts,_=cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            for c in cnts:
                x=600
                if cv2.contourArea(c)>x:
                    x,y,w,h=cv2.boundingRect(c)
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
                    cv2.putText(frame,("DETECT"),(10,180),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,0),2)
                    blue = 1 
            if red:
                RED.publish(1)
            else :
                RED.publish(0)
            if green:
                GRE.publish(1)
            else :
                GRE.publish(0)
            if blue:
                BLU.publish(1)
            else :
                BLU.publish(0)
            shouldread = 0
        
            cv2.imshow("Detected",frame)
        else :

            cv2.imshow("FRAME",frame)
        if cv2.waitKey(1)&0xFF==27:
            break
    cap.release()
    cv2.destroyAllWindows()
