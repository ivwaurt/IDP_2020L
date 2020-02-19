import numpy as np
import cv2 as cv
#Import video
cap = cv.VideoCapture("testvideo3.mp4")


while(cap.isOpened()):
    
    ret,frame = cap.read()
    output = frame
    
    #Quit if no frame
    if frame is None:
        break
    
    #Convert to grey and blur
    grey = cv.cvtColor(frame,cv.COLOR_RGB2GRAY)
    grey = cv.medianBlur(grey,21)
    
    #Find circles
    circles = cv.HoughCircles(
        image=grey,
        method=cv.HOUGH_GRADIENT,
        dp=1,
        minDist=30,
        param1=80,
        param2=40,
        minRadius=10,
        maxRadius=0
    )
    
    detected_circles = np.uint16(np.around(circles))
    
    #Mark detected circles
    for (x,y,r) in detected_circles[0,:]:
        try:
            cv.circle(output,(x,y),r,(0,255,0),3)
        except:
            pass
    
    #Show output
    cv.imshow('output',output)

    #Press q to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv.destroyAllWindows()
        break



	