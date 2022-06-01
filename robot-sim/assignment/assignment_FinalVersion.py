from __future__ import print_function

import time
from sr.robot import *

"""
Assignment 01 python script

The code should make the robot:
	- 1) find and grab the closest silver marker (token) 
	- 2) avoid collision with golden marker (token)
	- 3) move the marker behind the robot (move robot clockwise 180 degree)
	- 4) come back to original position (move robot counter-clockwise 180 degree)
	- 5) start again from 1
	

The method see() of the class Robot returns an object whose attribute info.marker_type may be MARKER_TOKEN_GOLD or MARKER_TOKEN_SILVER,
depending of the type of marker (golden or silver). 
This robot can do:

1- retrieve the distances and the angle of the closest silver marker and golden markers. If no silver marker is detected, the robot should move forward and rotate in order to find a silver marker and avoid collision with the golden markers.

2- drive the robot towards the silver marker and grab it

3- move the silver marker behind by rotating robot 180 degree clockwise. when done, release the marker by using the method release() of the class Robot.

4- drive the robot counter clockwise about 180 degree to its original position and move forward.

5- start again from 1


"""

a_th = 2.0
""" float: Threshold for the control of the linear distance"""

d_th = 0.4
""" float: Threshold for the control of the orientation"""

R = Robot()
""" instance of the class Robot"""

def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0
   

def find_token():
    dist = 100
    markers = R.see()
    
    for m in markers:	
	if m.dist < dist:
		dist=m.dist
		rot_y=m.rot_y
		marker=m.info.marker_type
	
    if dist==100:
	return -1, -1, -1
    else:
   	return dist, rot_y, marker

def grab_silver_token():
	while 1:

		dist, rot_y, marker = find_token()
		print("marker name: {0} distance: {1} rot_y: {2} ".format( marker, dist, rot_y))

		if dist < d_th and marker == MARKER_TOKEN_SILVER: # if we are close to the SILVER token, we try grab it.
			print("Found it!")
			
			if R.grab(): # if we grab the token, we move the robot clockwise, we release the token, and we go back to the initial position
	    			print("Gotcha!")
				turn(-30, 2)
				drive(40,0.5)
				R.release()
				print("Token released!")
				drive(-40,0.5)
				turn(30, 2)
				drive(30,0.5)
			continue
		
		elif -a_th <= rot_y <= a_th and marker is MARKER_TOKEN_SILVER: # if the robot is well aligned with the token, we go forward
			print("Heading towards silver token!.")
			drive(10, 1.5)
		
		elif -a_th > rot_y > -90 and marker is MARKER_TOKEN_SILVER:
	# if the robot is not well aligned with the token, we move it on the left or on the right
			print("Left a bit...")
			turn(-3, 0.5)	
		elif a_th < rot_y < 90 and marker is MARKER_TOKEN_SILVER:
			print("Right a bit...")
			turn(+3, 0.5)
	# we modify the value of the variable silver, so that in the next step we will look for the other type of token
		else:
			print("I don't see any silver token!!")
			drive(30, 0.5)
			break

def Sector_min(dist, rot_y_min, rot_y_max):
    """this method return the distance of the nearest golden token in the circular sector around the robot specified by rot_y_min, rot_y_max angles and the radius dist"""
    S=[]  
    for token in R.see():
        if token.dist < dist and rot_y_min <= token.rot_y <= rot_y_max and token.info.marker_type == MARKER_TOKEN_GOLD:
            S.append([token.dist , token.rot_y])  
    try: 
        """the try - except function is used to handle the case when there is no golden token in the circular sector"""     
        S_min = min(S , key=lambda x: x[0])
    except:
        S_min=[1000, 1000, 1000]
    return S_min[0]

def avoid_collision():
    """this method is used to avoid the arena's boundaries composed by golden token"""
    Ss=[Sector_min(10, -15, 15), Sector_min(10, -70, -15), Sector_min(10, 15, 70)]
    if min(Ss) == Ss[0] and Sector_min(10, -100, -80)-Sector_min(10, 80, 100) < -0.5 and Ss[0]<1.8:
        """in this case the robot has a near part of the boundaries in front of him and at his left, so to avoid it it has to turn right"""
        print("Boundry wall at my left and in front of me")
        turn(10, 1)
    elif min(Ss) == Ss[0] and Sector_min(10, -100, -80)-Sector_min(10, 80, 100) > 0.5 and Ss[0]<1.8:
        """in this case the robot has a near part of the boundaries in front of him and at his right, so to avoid it it has to turn left"""
        print("Boundry wall at my right and in fron of me")
        turn(-10, 1)   
    elif min(Ss) == Ss[0] and -0.5 < Sector_min(10, -100, -80)-Sector_min(10, 80, 100) < 0.5 and Ss[0]<1.8:
        """in this case the robot has a near part of the boundaries in front of him and the distance between the robot and the boundarie at his left and right is almost the same,
        so to avoid the beginning of a infinity loop where the robot turn right and then left i've decided to consider a different circular sector for the boundaries at the left and the right more
        closer to the driving direction"""
        if Sector_min(10, -80, -60)-Sector_min(10, 60, 80) > 0.5:
            print("Boundry wall at my right and in fron of me")
            turn(-10, 1)
        elif Sector_min(10, -80, -60)-Sector_min(10, 60, 80) < -0.5:
            print("Boundry wall at my left and in fron of me")
            turn(10, 1)             
    elif min(Ss) == Ss[1] and Ss[1]<0.8 :
        """ in this case the boundarie is near the robot at his left so it has to turn right a bit"""
        print("Boundry wall at my left")
        turn(15, 1)
    elif min(Ss) == Ss[2] and Ss[2]<0.8:
        """ in this case the boundarie is near the robot at his left so it has to turn right a bit"""
        print("Boundry wall at my right")
        turn(-15, 1)
    else:
        """ in this case the robot is far enough from the boundarie and it can drive straight"""
        drive(15, 0.5)
        print("driving")


def main():
	while 1:
		grab_silver_token()
		avoid_collision()

main()
