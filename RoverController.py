from __future__ import division
import cv2
#to show the image
#from matplotlib import pyplot as plt
import numpy as np
from math import cos, sin
from PIL import Image
import picamera
from time import sleep
import serial
import RPi.GPIO as GPIO

GPIO.setwarnings(False);

def SetAngleX(pin,angle):
	duty = angle / 18 + 2
	GPIO.output(pin, True)
	pwmX.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(pin, False)
	pwmX.ChangeDutyCycle(0)



#pin 7 is for servo that rotates on X axis


motX = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motX, GPIO.OUT)
pwmX=GPIO.PWM(motX, 50)
pwmX.start(0)
currentAngle = 50
SetAngleX(motX,currentAngle)
#GPIO.setmode(GPIO.BOARD)
ser = serial.Serial("/dev/ttyS0")
ser.baudrate = 9600
#enablePin = 18
#GPIO.setup(enablePin,GPIO.OUT)
#pin 13 is for servo that rotates on Y axis
#pin 7 is for servo that rotates on X axis


green = (0, 255, 0)


def sendSerial(value):
    #GPIO.output(enablePin,GPIO.HIGH)
    print("sending serial" + str(value))
    a = str(value)
    ser.write(a.encode('UTF-8'))
    #sleep(1)
    #GPIO.output(enablePin,GPIO.LOW)

def show(image):
    plt.figure(figsize=(10, 10))
    plt.imshow(image, interpolation='nearest')

def overlay_mask(mask, image):
    rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    img = cv2.addWeighted(rgb_mask, 0.5, image, 0.5, 0)
    return img

def find_biggest_contour(image):
    image = image.copy()
    _, contours, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Isolate largest contour
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
  #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

    mask = np.zeros(image.shape, np.uint8)
#cv2.drawContours(mask, [biggest_contour], -1, 255, -1)
    return contours, mask

def circle_contour(image, contour):
    image_with_ellipse = image.copy()
    ellipse = cv2.fitEllipse(contour)
    cv2.ellipse(image_with_ellipse, ellipse, green, 2,cv2.LINE_AA)
    return image_with_ellipse

def find_strawberry(image):
    try:
        global currentAngle
        screenHeight, screenWidth,_ = image.shape
        screenMidX = screenWidth/2
        screenMidY = screenHeight/2
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #max_dimension = max(image.shape)
        #scale = 700/max_dimension
        #image = cv2.resize(image, None, fx=scale, fy=scale)
        image_blur = cv2.GaussianBlur(image, (7, 7), 0)
        image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
        min_red = np.array([0, 100, 80])
        max_red = np.array([10, 256, 256])
        #layer
        mask1 = cv2.inRange(image_blur_hsv, min_red, max_red)
        min_red2 = np.array([170, 100, 80])
        max_red2 = np.array([180, 256, 256])
        mask2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)
        mask = mask1 + mask2
        #mask = mask1
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        #mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        #mask_clean = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)
        #cv2.imshow("Mask",mask_clean)
        # cv2.waitKey(0)
        cnts, contours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        
        
        #big_strawberry_contour, mask_strawberries = find_biggest_contour(mask_clean)
        largest_contour = sorted(contours, key=cv2.contourArea,reverse = True)
        if(len(largest_contour) > 0):
            cnts = largest_contour[0]
            cv2.drawContours(image, cnts, -1, (0, 255, 0), 2)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            #print("hello")
            #cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            centerballon = cv2.moments(cnts)
            cX = int(centerballon["m10"] / centerballon["m00"])
            cY = int(centerballon["m01"] / centerballon["m00"])
            cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(image, "X: "+str(cX), (cX - 20, cY - 40),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(image, "Y: "+str(cY), (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            overlay = overlay_mask(mask, image)
            circled = circle_contour(overlay, cnts)
            print("Shape centerX/centerY: " + str(cX)+"/"+ str(cY))
            print("Area: " + str(cv2.contourArea(cnts)));
            print("Image centerX/centerY: " + str(screenMidX) +"/" + str(screenMidY))
            
            diffY = screenMidY - cY
            print(str(diffY))
            if(abs(diffY) > 40):
                if(diffY > 0):
                    # move down
                    if(currentAngle < 90):
                        currentAngle = currentAngle + 10
                        SetAngleX(motX,currentAngle)
                if(diffY < 0):
                    #if(currentAngle > 0):
                        currentAngle = currentAngle - 10
                        SetAngleX(motX,currentAngle)
            else:
                print("In Range Y")
                #SetAngleX(motX,currentAngle)
            #SetAngleX(motX,currentAngle)    
            diff = screenMidX - cX
            
            if(abs(diff) > 150):
                if(diff > 0):
                    sendSerial('a')
                    sleep(0.2)
                    print("Move left")
                if(diff < 0):
                    sendSerial('b')
                    sleep(0.2)
                    print("Move right")
            if (abs(diff) <= 150):
                sendSerial('c')
                print("In Range")
        if(len(largest_contour) == 0):
            sendSerial('d')
            print('No Red Color')
            currentAngle = 50
            SetAngleX(motX,currentAngle)
            #image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
                    
            #show(circled)
            #bgr = cv2.cvtColor(circled, cv2.COLOR_RGB2BGR)
    except:
        print('Exception')    
        
    return image

#cap = cv2.VideoCapture(0)
#while(True):
 #   ret, captured_frame = cap.read()
  #  output_frame = captured_frame.copy()
   # captured_frame_bgr = cv2.cvtColor(captured_frame, cv2.COLOR_BGRA2BGR)




    
    
cam = picamera.PiCamera()
while(True):
    #cam.start_preview(fullscreen=False,window=(0,0,640,480))
    cam.capture('/home/pi/Desktop/t.jpg')
    image = cv2.imread('/home/pi/Desktop/t.jpg')
    result = find_strawberry(image)
    cv2.imwrite('/home/pi/Desktop/out.jpg', result)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
        break;


 

 

cam.stop_preview()
ser.close()
#cap.release()
#cv2.destroyAllWindows()    

