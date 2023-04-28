import cv2 #Import CV2 
import numpy as np #Import Numpy
import sys
import time
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
MQTT_SERVER = "broker.hivemq.com"
MQTT_PATH = "resulttopic"
MQTT_PATH_1 = "starttopic"
sleeptime=1
Relay_solenoid_sw2=12
Direction_pin = 36
Enable_servo= 37
Direction_pin_1 = 38
Enable_servo_1= 40
position_x=0
position_y=0
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Relay_solenoid_sw2, GPIO.OUT) # Set pin 12 to be an input pin and set
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(Direction_pin, GPIO.OUT)# 36 for Motor Direction
GPIO.setup(Enable_servo, GPIO.OUT)#37 for Motor Running Pin
GPIO.setup(Direction_pin_1, GPIO.OUT)# 38 for Motor Direction
GPIO.setup(Enable_servo_1, GPIO.OUT)#40 for Motor Running Pin
GPIO.output(16, GPIO.HIGH)
GPIO.output(18, GPIO.HIGH)
camera_flag=0
frame_count=0
##################CAMERA INIT##################################
cap = cv2.VideoCapture(0) #Catpure video from camera
Ht = 320 #Defined Height of frame
Wd = 480 #Defined Width of Frame
cap.set(3, Wd) #Set frame Width
cap.set(4, Ht) #Set frame height
_,frame = cap.read() #Store captured frame of camera to variable "frame"
rows, cols, ch = frame.shape #Get frame size 
x_medium = int(cols / 2) #Initialize horizontal position 
y_medium = int(rows / 2) #Initialize vertical positon

x_center = int(cols / 2) #Initialize Horizontal center position
y_center = int(rows / 2) #Initialize Vertical center position
x_position = 90 # centre posito of servo 
y_position = 90 # centre posito of servo
x_band = 50
y_band = 50
position_flag=0
##################################################################
old_position=0
old_position_y=0
frame_loop_check=0
GPIO.output(Direction_pin, GPIO.LOW)
GPIO.output(Enable_servo, GPIO.LOW)
def translate(value, leftMin, leftMax, rightMin, rightMax):
    return (value-leftMin)*(rightMax-rightMin)/(leftMax-leftMin)+rightMin
##################################################################
def compare_strings(str1):
    if(str1.find("b'1'")):
        return 1
    else:
        return 0
     
    return count1 == count2
# The callback for when the client receives a connect response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # on_connect() means that if we lose the connection and reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_PATH)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    # global position_flag
    # position_flag=1
    global camera_flag
    print(str(msg.payload))
    result=str(msg.payload)
    #print(result.find(":0"))
    if(result.find(":0")!=-1):
        camera_flag=1
    
    
    # more callbacks, etc

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_SERVER, 1883, 60)
 

time.sleep(3)
GPIO.output(16, GPIO.LOW)
GPIO.output(18, GPIO.LOW)
while (1):

        #client.loop_forever() #   
        client.loop_start()
        ###############CAMERA CODE######################################
        _, frame1 = cap.read() #Store Video snap in varialble "frame1"
        frame2 = cv2.flip(frame1,-1) # Flip image vertically
        frame2 = cv2.flip(frame1, 0) # flip image vertically
        hsv_frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
        #blurred_frame = cv2.GaussianBlur(frame1, (5, 5), 0)
        #Red Colour
        low_red = np.array([163,74,30]) #low HSV value for Red objects
        high_red = np.array([179,255,255]) # High HSV value for Red objects
        red_mask = cv2.inRange(hsv_frame2,low_red,high_red) # Apply Masking to image using low & Hign red masking value
        red = cv2.bitwise_and(frame2,frame2,mask=red_mask) # Anding of original frame & 
        #Contors
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # Findig Contours
        contours = sorted(contours_red, key=lambda x:cv2.contourArea(x), reverse=True) # Arrange Contours in Assending
        for cnt in contours: # Draw rectangle on First contors on image
            (x,y,w,h) = cv2.boundingRect(cnt)
            cv2.rectangle(frame2, (x , y) , (x + w, y + h) , (0, 255, 0), 2) # Getting Position of rectangle & line colour & thickness
            break # Break loop to draw only one rectangle. if comment we get all red object rectangle
        for cnt in contours:
            (x,y,w,h) = cv2.boundingRect(cnt)
            x_medium = int((x + x + w) / 2) # Checking horizontal center of red object & save to variable
            y_medium = int((y + y + h) / 2) # Checking Vertical center of red object & save to variable
            break
        cv2.line(frame2, (x_medium, 0), (x_medium, Ht), (0, 255, 0), 2) #Draw horizontal centre line of red object
        cv2.line(frame2, (0, y_medium), (Wd, y_medium), (0, 255, 0), 2) #Draw Vertical centre line of red object
        cv2.imshow("IN Frame", frame2) #Printing frame with rectangle &  lines

        position_x=int(translate(x_medium, 0, 620, 0, 1200)) # 50.0)  
        position_y=int(translate(y_medium, 0, 620, 0, 1300)) # 50.0) 
        if old_position>position_x+50:
            print("Moving Forward_X")
            for i in range(old_position,position_x):
                    GPIO.output(Direction_pin, GPIO.HIGH)##Rotate ClockWise
                    GPIO.output(Enable_servo, GPIO.HIGH)
                    
                    time.sleep(0.001)
                    GPIO.output(Enable_servo, GPIO.LOW)
                    time.sleep(0.001)
                           
            print("****************************************")
        if old_position<position_x-50:
            print("Moving backward_x")
            for i in range(position_x,old_position):
                    GPIO.output(Direction_pin, GPIO.LOW)##Rotate Anti Clock wise
                    GPIO.output(Enable_servo, GPIO.HIGH)
                    time.sleep(0.001)
                    GPIO.output(Enable_servo, GPIO.LOW)
                    time.sleep(0.001)
            print("****************************************")
        ###############END OF CAMERA CODE################################
        if camera_flag:
            camera_flag=0
            print("x =", x_medium , "y =", y_medium) 
            print("x =", position_x , "y =", position_y) 
            #x-axis motor forward and reverse motor control
           
            ########################################################################
            #x-axis motor forward and reverse motor control
            print("Moving Forward_y")
            for i in range(0,position_y):
                    GPIO.output(Direction_pin_1, GPIO.HIGH)##Rotate ClockWise
                    GPIO.output(Enable_servo_1, GPIO.HIGH)
                    
                    time.sleep(0.001)
                    GPIO.output(Enable_servo_1, GPIO.LOW)
                    time.sleep(0.001)
            GPIO.output(Relay_solenoid_sw2, GPIO.HIGH)
            time.sleep(2)
            GPIO.output(Relay_solenoid_sw2, GPIO.LOW)
            time.sleep(5)
            print("Moving backward_y")
            for i in range(0,position_y):
                    GPIO.output(Direction_pin_1, GPIO.LOW)##Rotate Anti Clock wise
                    GPIO.output(Enable_servo_1, GPIO.HIGH)
                    time.sleep(0.001)
                    GPIO.output(Enable_servo_1, GPIO.LOW)
                    time.sleep(0.001)
            print("****************************************")
           
            #########################################################################
        old_position=position_x
        key = cv2.waitKey(1)
        if key == 27:
            #kit.servo[0].angle =(90) 
            #kit.servo[1].angle =(90) 
            print("key", key)    
            break
            
            
            



# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
#client.loop_forever() #   

cv2.destroyAllWindows()
cap.release()

		
		

		
