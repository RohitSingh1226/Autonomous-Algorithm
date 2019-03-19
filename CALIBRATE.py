import cv2
import numpy as np
cam=cv2.VideoCapture(1)
def nothing(x):
    pass
#canny+Circle+HSV
cv2.namedWindow('CannyEDGE')
cv2.resizeWindow('CannyEDGE',600,600)
cv2.createTrackbar('CannyLow','CannyEDGE',0,1000,nothing)
cv2.createTrackbar('CannyHigh','CannyEDGE',0,1000,nothing)
cv2.createTrackbar('H','CannyEDGE',0,255,nothing)
cv2.createTrackbar('S','CannyEDGE',0,255,nothing)
cv2.createTrackbar('V','CannyEDGE',0,255,nothing)
cv2.createTrackbar('HM','CannyEDGE',0,255,nothing)
cv2.createTrackbar('SM','CannyEDGE',0,255,nothing)
cv2.createTrackbar('VM','CannyEDGE',0,255,nothing)
cv2.createTrackbar('DK','CannyEDGE',1,100,nothing)
cv2.createTrackbar('OP','CannyEDGE',1,100,nothing)
pso=1
exitV=0
kernel = None
def calibrationBall(image,H,S,V,HM,SM,VM):
    global lower
    global upper
    #print('Entered')
    lower=np.array([H,S,V])
    upper=np.array([HM,SM,VM])
    inRange=cv2.inRange(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV),lower,upper)
    #print('Processing')
    cv2.imshow('frame',inRange)
def cannyEdge(image,val1,val2):
    edges = cv2.Canny(image,val1,val2)
    ret,thresh=cv2.threshold(edges,127,255,cv2.THRESH_BINARY_INV)
    cv2.imshow('Edges',edges)
    cv2.imshow('Edges1',thresh)
    contour_FIND_and_LABEL(edges,thresh)
def dilate_image(image,val1):
    global kernel
    kernel=np.ones((val1,val1),np.uint8)
    dilation = cv2.dilate(image,kernel,iterations = 1)
    return(dilation)
def morphological_open(image,val1):
    kernel=np.ones((val1,val1),np.uint8)
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return(opening)
def contour_FIND_and_LABEL(image,image2):
    global frame
    _, contours, _ = cv2.findContours(image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #_, contours1, _ = cv2.findContours(image2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if (len(contours)!=0):          
        cnt=max(contours,key=cv2.contourArea)
        '''cnt1=max(contours1,key=cv2.contourArea)
        imgx=cv2.drawContours(frame, [cnt], 0, (0,255,0), 3)
        imgy=cv2.drawContours(frame, [cnt1], 0, (0,255,0), 3)
        cv2.imshow('FrameImage1',imgx)
        cv2.imshow('FrameImage',imgy)'''
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        area = cv2.contourArea(cnt)
        cv2.circle(frame,center,radius,(0,255,0),2)
        cv2.putText(frame,'Ball detected',(center[0]+10,center[1]+10), cv2.FONT_HERSHEY_SIMPLEX, 0.8,(0,0,255),2,cv2.LINE_AA)
        Navigate_Autonomously(frame,center,area)
    else:
        cv2.imshow('FrameImage',frame)
def Navigate_Autonomously(frame,center,area):
    global exitV
    #print(center[0])
    if(area>500):
        print('Destination reached')
        String_V='Destination reached'
        exitV=1
    if(center[0]<=210):
        print('Turning Left')
        String_V='Turning Left'
    elif(center[0]>210 and center[0]<=420):
        print('Moving forward')
        String_V='Moving forward'
    elif(center[0]>420):
        print('Turn Right')
        String_V='Turning right'
    cv2.putText(frame,String_V,(0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,0),2,cv2.LINE_AA)
    cv2.putText(frame,'Distance to traverse: '+str(10000/(area+1)),(0,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,0),2,cv2.LINE_AA)
    cv2.imshow('FrameImage',frame)
    
#Notes: IMAGE WIDTH:  480X640X3
#rangeofball=
#Inmyroom: 28 83 112 72 163 188
lower=np.array([28,83,112])
upper=np.array([72,163,188])
pos=1
#kernelg=np.ones([5,5],dtype=np.uint8)
while True:
    k=cv2.waitKey(1)
    if k==ord('q'):
        print('Quiting')
        break
    val1=cv2.getTrackbarPos('CannyLow','CannyEDGE')
    val2=cv2.getTrackbarPos('CannyHigh','CannyEDGE')
    H=cv2.getTrackbarPos('H','CannyEDGE')
    S=cv2.getTrackbarPos('S','CannyEDGE')
    V=cv2.getTrackbarPos('V','CannyEDGE')
    HM=cv2.getTrackbarPos('HM','CannyEDGE')
    SM=cv2.getTrackbarPos('SM','CannyEDGE')
    VM=cv2.getTrackbarPos('VM','CannyEDGE')
    dil_kernel=cv2.getTrackbarPos('DK','CannyEDGE')
    opend=cv2.getTrackbarPos('OP','CannyEDGE')
    _,frame=cam.read()
    if k==ord('c'):
        pos=0
    if k==ord('r'):
        pos=2
    if k==ord('s'):
        pos=1
    if pos==0:
        calibrationBall(frame,H,S,V,HM,SM,VM)
    if lower!=[] and pos==0:
        framex=cv2.GaussianBlur(frame,(5,5),0)
        inRange=cv2.inRange(cv2.cvtColor(framex,cv2.COLOR_BGR2HSV),lower,upper)
        cv2.imshow('inRange1',inRange)
        opening=morphological_open(inRange,opend)
        cv2.imshow('Opend',opening)
        dilation=dilate_image(opening,dil_kernel)
        cannyEdge(dilation,val1,val2)
    elif lower!=[] and pos==2:
        inRange=cv2.inRange(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV),lower,upper)
        inRange=dilate_image(inRange,dil_kernel)
        cannyEdge(inRange,val1,val2)
    else:
        inRange=cv2.inRange(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV),lower,upper)
        #cv2.imshow('inRange1',inRange)
        opening=morphological_open(inRange,opend)
        #cv2.imshow('Opend',opening)
        dilation=dilate_image(opening,dil_kernel)        
        cannyEdge(dilation,val1,val2)
        #cv2.imshow('Filtered',inRange)
    #cv2.imshow('actual',frame)

cam.release()
