import time
import math
import sys
sys.setrecursionlimit(1000000)
import serial
import cv2

import numpy as np
import string
import pynmea2
import RPi.GPIO as GPIO
import smbus
from haversine import haversine


address=0x09
bus=smbus.SMBus(1)

def goforward(time1):
    bus.write_byte(address, 255)
    time.sleep(.3)
    bus.write_i2c_block_data(address, 1, [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,128,2,128,0,0,0,0])
    time.sleep(time1)
    bus.write_byte(address, 255)
    time.sleep(.3)
    bus.write_i2c_block_data(address, 1,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    time.sleep(.3)
    number = bus.read_i2c_block_data(address,254,8)
    time.sleep(.3)
    print("forward")


def goback(time1):
    bus.write_byte(address, 255)
    time.sleep(.3)
    bus.write_i2c_block_data(address, 1, [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,128,1,128,0,0,0,0])
    time.sleep(time1)
    bus.write_byte(address, 255)
    time.sleep(.3)
    bus.write_i2c_block_data(address, 1,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    time.sleep(.3)
    number = bus.read_i2c_block_data(address,254,8)
    time.sleep(.3)
    print("back")


def goright(time2):
     bus.write_byte(address, 255)
     time.sleep(.3)
     bus.write_i2c_block_data(address, 1,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,128,1,128,0,0,0,0])
     time.sleep(time2)
     bus.write_byte(address, 255)
     time.sleep(.3)
     bus.write_i2c_block_data(address, 1,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
     time.sleep(.3)
     number = bus.read_i2c_block_data(address,254,8)
     time.sleep(.3)
     print("right")


def goleft(time3):
     bus.write_byte(address, 255)
     time.sleep(.3)
     bus.write_i2c_block_data(address, 1,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,128,2,128,0,0,0,0])
     time.sleep(time3)
     bus.write_byte(address, 255)
     time.sleep(.3)
     bus.write_i2c_block_data(address, 1,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
     time.sleep(.3)
     number = bus.read_i2c_block_data(address,254,8)
     time.sleep(.3)
     print("left")



def getGPS():
    f=open('GPS_val1.txt','r')
    s=f.readlines()
    #print(len(s))                               ## Lat longitude
    f.close()
    last_line = s[-1]
    arr=last_line.split(' ')
    gps_x = float(arr[0])
    gps_y = float(arr[1])
    print(gps_x,gps_y)
    return [gps_x,gps_y]

'''
def getGPS():
    x = float(input("Enter X : "))
    y = float(input("Enetr Y : "))
    return [x,y]
'''

def getAngle():
    global finalx, finaly
    turn_flag=0
    print(1)
    ultra_value = getUltrasonic(ser)
    print(2)
    while(ultra_value!=0):
        goback(1)
        print('Obstace detected')
        if(ultra_value==1):
                goleft(2)
        elif(ultra_value==3):
                goright(2)
        ultra_value = getUltrasonic(ser)
        turn_flag=1

    if(turn_flag):
       # EXTRA LEFT CAN BE ADDED
        goforward(2)



    print("Before")
    anum = getGPS()

    oldx=anum[0]
    oldy=anum[1]

########  CHEKING OBSTACLE 5 TIMES  ##############

    for i in range (10) :
        turn_flag=0
        goforward(1)
        check_gps = getGPS()
    #d = (  (check_gps[0]-finalx)**2 + (check_gps[1]-finaly)**2   )**0.5
        d = haversine(check_gps, [finalx, finaly])
        print ('distance is : ',d)
        if(d <= 0.002):
            print(d)
            return -1111
        ultra_value = getUltrasonic(ser)
        while(ultra_value!=0):
           goback(1)
           print('Obstacle Detected')
           if(ultra_value==1):
              goleft(2)
           elif(ultra_value==3):
              goright(2)
           turn_flag=1
           ultra_value = getUltrasonic(ser)
        if(turn_flag):
           goforward(1)

        time.sleep(.5)


    print('After')
    anum1=getGPS()

   ######## CHECK FOR FINAL VALUE  ##############
    #if( ((anum1[0]-finalx)**2 + (anum[1]-finaly)**2)**0.5 <= 9.3392e-05):
    #return -1111


    currentx=anum1[0]
    currenty=anum1[1]
    print(finalx, finaly, oldx, oldy, currentx, currenty)
    # Vector made by (oldx, oldy) to (currentx, currenty)
    A = [currentx - oldx, currenty - oldy]
    #print(A)
    # Vector made by (oldx, oldy) to (finalx, finaly)
    B = [finalx - oldx, finaly - oldy]
    #print(B)
    # Vector made by (currentx, currenty) to (finalx, finaly)
    # A + C = B
    # Therefore, C = B - A
    C = [0, 0]
    C[1] = B[1] - A[1]
    C[0] = B[0] - A[0]

    ''' To find the angle between A and C '''
    ''' calculate the dot product of A and C/mag(A)*mag(C) '''

    costheta_numerator = A[0] * C[0] + A[1] * C[1]
    costheta_denominator = ((A[0] ** 2 + A[1] ** 2) ** 0.5) * ((C[0] ** 2 + C[1] ** 2) ** 0.5)

    if costheta_denominator == 0:
        mainfunction(1)
        print('denom 0')
        return

    costheta = costheta_numerator / costheta_denominator

    angle =int(math.degrees(math.acos(costheta)))

    # Cross Product
    sintheta_numerator = A[0] * C[1] - A[1] * C[0]
    sintheta_denominator = costheta_denominator
    sintheta = sintheta_numerator / sintheta_denominator

    sinangle = math.degrees(math.asin(sintheta))

    if sinangle > 0:
        angle= -1 * angle

    print( 'returning angle : ', angle)
    return angle


def setUltrasonic():
    port = '/dev/ttyACM1'
    ser1 = serial.Serial(port,9600)
    return ser1
'''
def getUltrasonic(ser1):
	return 0
'''
def getUltrasonic(ser1):
    port = '/dev/ttyACM1'
    ser1 = serial.Serial(port,9600)
    ser1.flushInput()
    #ser1.flushOutput()
    sum1=0
    sum2=0
    sum3=0
    flag=1
    flag_0=0
    while(flag):
        try:
            #########################   GARBAGE MAKES SLOW   ########### Prevent
            #for i in range (10):
                #data = ser1.readline()
            #print('garbage done')
            ############## DATA1 = RIGHT , DATA3 = LEFT ############
                for i in range(5):
                    all_data = (ser1.readline())
                    print('ultra')
#                    print(all_data)
                    all_data = all_data.split(b',')
                    data1 = int(all_data[0])
                    data2 = int(all_data[1])
                    data3 = int(all_data[2])
                    if(data1 == 0):
                        data1=6000
                    if(data2==0):
                        data2=6000
                    if(data3==0):
                        data3=6000
                    sum1+=data1
                    sum2+=data2
                    sum3+=data3
                data1 = sum1 / 5
                data2 = sum2 / 5
                data3 = sum3 / 5
                print( 'ultra : ', data1, data2, data3)
                flag=0
        except:
            flag=1

    ##############   BUFFERS TO BE RECALCULATED   ########################################

    if(data1 < 80 or data2 < 80):
        return 1
    elif(data3 < 80):
        return 3
    else:
        return 0


def getCurrentAngle():
    port = "/dev/ttyACM0"
    count=0
    sum=5
    flag=1
    ser1=serial.Serial(port,9600)
    ser1.flushInput()
    #ser1.flushOutput()
    while(flag):
        try:
            data11 = ser1.readline()
            data11 = float(data11)
            print ("compass : ",data11)
            flag=0
            return (data11)
        except:
            print("exception here")
            flag=1


def turnRover(angle):
    try:
        initial=getCurrentAngle()
        initial=getCurrentAngle()
        final = (initial+angle)%360
        print( '\n\nInitial : ', initial)
        print ('Angle to turn : ', angle)
        print ('Final : ', final)
        print ('\n\n')
        flag=0
        if(angle<0):
                 while True:
                        dummy=getCurrentAngle()
                        if( abs(dummy-final)   <= 12 and abs(dummy + final)  >= 12 ):
                            print('Turning complete')
                            break
                        else:
                            goright(.5);
                            time.sleep(.1)
        else:
                while True:
                        dummy1=getCurrentAngle()
                        if(  abs(dummy1-final)   <= 12 and abs(dummy1+final) >= 12 ):   # Threshold of 12
                            print('Turning complete')
                            break
                        else:
                            goleft(.5)
                            time.sleep(.1)
    except:
        print('exception here')


def radialfunction(lat,lon,angle,distance):
	latfinal=math.asin(math.sin(lat)*math.cos(distance)+math.cos(lat)*math.sin(distance)*math.cos(angle))
        dlon=math.atan2(math.sin(angle)*math.sin(distance)*math.cos(lat),math.cos(distance)-math.sin(lat)*math.sin(latfinal))
        lonfinal=abs(lon-dlon+math.pi,2*math.pi)-math.pi
        return [latfinal,lonfinal]    
def getGoals(n):
    global finalx, finaly, checkpoints
    checkpoints = []
    x = float(input('Enter x : '))
    y = float(input('Enter y : '))
    cood = (x,y)
    checkpoints.append(cood)
    distance=
    


def checkObject():
    #capturing video through webcam
    cap=cv2.VideoCapture(0)
    #print(1)
    start = time.time()
    box = 0
    bottle = 0 
    disc = 0
    print('looking for object')
    try:
        for i in range (10):
            #st=time.time()
            _, img = cap.read()
            hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        #red ka according to google
            red_lower=np.array([136,87,111],np.uint8)
            red_upper=np.array([180,255,255],np.uint8)

        #blue ka according to google 
            blue_lower=np.array([99,115,150],np.uint8)
            blue_upper=np.array([110,255,255],np.uint8)
        
        #yellow ka acc to google 
            yellow_lower=np.array([22,60,200],np.uint8)
            yellow_upper=np.array([60,255,255],np.uint8)

        #sabka range nikala
            red=cv2.inRange(hsv, red_lower, red_upper)
            blue=cv2.inRange(hsv,blue_lower,blue_upper)
            yellow=cv2.inRange(hsv,yellow_lower,yellow_upper)
        
        # Dilation karunga      
            \        #red ke liye
            (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area>500):
                    x,y,w,h = cv2.boundingRect(contour)
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(img,"RED color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
                    print('Box')
                    box = 1

                
        #blue ke liye
            (_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area>500):
                    x,y,w,h = cv2.boundingRect(contour) 
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(img,"Blue color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0))
                    print('Bottle')
                    bottle = 1
        #yellow ke liye
            (_,contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area>500):
                    x,y,w,h = cv2.boundingRect(contour) 
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(img,"yellow  color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
                    print('Disc')
                    disc = 1 
               
               
            
            #cv2.imshow("Color Tracking",img)
            #stop_t=time.time()
            #print('Frame rate: ',1/(stop_t-st))
        
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break

            return [box, bottle, disc]
         
    except:
        print('exceptoin')
        return [0,0,0]


def mainfunction(ser):
    angleneed=getAngle()
    if(angleneed==-1111):
        return 0
    time.sleep(.4)
    turnRover(angleneed)
    time.sleep(.2)
    return 1

finalx = 0.0
finaly = 0.0
checkpoints = []

n = int(input('Enter number of checkpoints : '))
getGoals(n)
ser = setUltrasonic()
for i in range(n):
    finalx = checkpoints[i][0] 
    finaly = checkpoints[i][1]
    finalflag=1
    while(finalflag):
        finalflag=mainfunction(ser)
    print('Checkpoint ', i+1, ' reached.')

    obflag=1
    flag_final=0
    count=0
    box = 0
    bottle = 0
    disc = 0
    for j in range(8):
        objects = checkObject()
        if(objects[0]==0 and objects[1]==0 and objects[2]==0):
            count+=1

        if(objects[0]==1):
            print('found a box')
	    box+=1

        if(objects[1]==1):
           print('found a bottle')
	   bottle+=1
 
        if(objects[2]==1):
           print('found a disc')
	   disc+=1

        if(count==5):
            print('Nothing found')
            count=0
        goleft(2)

    if(box!=0 or bottle!=0 or disc==0 ):
		break;


print('\n\n#####  Objects Found  ###########/n')
print 'Box : ' box
print 'Bottle : ', bottle
print 'Disc : ', disc         
print('You have completed task')


