import cv2   
import numpy as np
#import time
#capturing video through webcam
cap=cv2.VideoCapture(0)
try:
    while(1):
        #st=time.time()
        #print("Begin")
        _, img = cap.read()
        
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

	#red ka according to google 
        red_lower=np.array([136,87,111],np.uint8)
        red_upper=np.array([180,255,255],np.uint8)

	#blue ka according to google 
        blue_lower=np.array([141,59,0],np.uint8)
        blue_upper=np.array([205,161,57],np.uint8)
        lower_sky = np.array([20,108,93])
        upper_sky = np.array([182,174,186])
        
	
	#yellow ka acc to google 
        yellow_lower=np.array([22,60,200],np.uint8)
        yellow_upper=np.array([60,255,255],np.uint8)

	#sabka range nikala
        red=cv2.inRange(hsv, red_lower, red_upper)
        blue=cv2.inRange(hsv,blue_lower,blue_upper)
        #mask2 = cv2.inRange(hsv,lower_sky,upper_sky)
        #masknosky=cv2.bitwise_not(mask2,mask2,mask = mask2)
        #finalblue = cv2.bitwise_and(masknosky,blue)
        cv2.imshow('Final blue',blue)
        #res1=cv2.bitwise_and(img, img, mask = finalblue)
        yellow=cv2.inRange(hsv,yellow_lower,yellow_upper)
	
	# Dilation karunga  	
        kernal = np.ones((5 ,5), "uint8")
        red=cv2.dilate(red, kernal)
        res=cv2.bitwise_and(img, img, mask = red)

        blue=cv2.dilate(blue,kernal)
        
        
        
        

        yellow=cv2.dilate(yellow,kernal)
        res2=cv2.bitwise_and(img, img, mask = yellow)   


	#red ke liye
        (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>100):
                x,y,w,h = cv2.boundingRect(contour)
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(img,"RED color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
                print('Box')
			
	#blue ke liye
        (_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>100):
                x,y,w,h = cv2.boundingRect(contour)	
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(img,"Blue color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0))
                print('Bottle')

	#yellow ke liye
        (_,contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>100):
                x,y,w,h = cv2.boundingRect(contour)	
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(img,"yellow  color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
                print('Disc')
           
           
    	
        cv2.imshow("Color Tracking",img)
        #stop_t=time.time()
        #cv2.imshow("red",res)
        #cv2.imshow("blue",res1)
        #cv2.imshow("yellow",res2)
        #print('Frame rate: ',(1/(stop_t-st)))
        '''except XERROR:
        print("retry")
        pass'''
       # if(img==red):
            #print('bottle')
       #print('Frame rate: ',1/(stop_t-st))

#add dilation if the there is no lag on rpi
       
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
     
except Exception as e:
    print(e)
