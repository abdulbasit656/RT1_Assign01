Python Robotics Simulator
================================

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).
Some of the arenas and the exercises have been modified for the Research Track I course

Installing and running
----------------------

The simulator requires a Python 2.7 installation, the [pygame](http://pygame.org/) library, [PyPyBox2D](https://pypi.python.org/pypi/pypybox2d/2.1-r331), and [PyYAML](https://pypi.python.org/pypi/PyYAML/).

When running python run.py <file>, you may be presented with an error: ImportError: No module named 'robot'. This may be due to a conflict between sr.tools and sr.robot. To resolve, symlink simulator/sr/robot to the location of sr.tools.

On Ubuntu, this can be accomplished by:

    Find the location of srtools: pip show sr.tools
    Get the location. In my case this was /usr/local/lib/python2.7/dist-packages
    Create symlink: ln -s path/to/simulator/sr/robot /usr/local/lib/python2.7/dist-packages/sr/

Once the dependencies are installed, simply run the `test.py` script to test out the simulator.
	
Robot API
---------

The API for controlling a simulated robot is designed to be as similar as possible to the [SR API][sr-api].
	
The Arena 
-----------------------------
![Robot Simulator](https://user-images.githubusercontent.com/17598805/171017018-df5fd141-dd3e-4a12-b20a-7692f44f59f9.png)

-----------------------------
In this Arena the robot starts its ride from the top left corner and, has to:

    Moves counterclockwisely
    Avoids golden tokens
    Grabs and releases behind it the silver tokens


Assignment Steps
-----------------------------
Assignment 01 python script

The code should make the robot:
	- 1) find and grab the closest silver marker (token) 
	- 2) avoid collision with golden marker (token)
	- 3) move the silver marker behind the robot (move robot clockwise 180 degree)
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

To run command
------------------------	
In order to run the assignment script in the simulator you have to insert this command on the shell, after entering in the directory	
 
```bash
$ python2 run.py assignment/assignment_v2.py
```
Implementation
------------------------
In this assignment I have implemented saveral functions to drive robot according to given task. 

### Motors ###

The simulated robot has two motors configured for skid steering, connected to a two-output [Motor Board](https://studentrobotics.org/docs/kit/motor_board). The left motor is connected to output `0` and the right motor to output `1`.

The Motor Board API is identical to [that of the SR API](https://studentrobotics.org/docs/programming/sr/motors/), except that motor boards cannot be addressed by serial number. So, to turn on the spot at one quarter of full power, one might write the following:

```python
R.motors[0].m0.power = 25
R.motors[0].m1.power = -25
```
**drive()**
The drive() function was created to allow the robot to move straight, it can go forward, giving to speedparameter a positive value, or it can go backward giving to speed parameter a negative value

    Arguments
        speed: the linear velocity that we want the robot to assume.
        seconds: the amount of seconds we want to drive.
    Returns
        None.
Code:

```python
def drive(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0
```
**turn()**
The turn() function turns the robot anticlockwise when ever it calls in code.
	
    Arguments
        speed: the angular velocity with which the robot rotates on itself.
        seconds: the interval of time in which the robot turns, rotating on itself.
    Returns
        None.
Code:

```python
def turn(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0
```
### Vision ###

To help the robot find tokens and navigate, each token has markers stuck to it, as does each wall. The `R.see` method returns a list of all the markers the robot can see, as `Marker` objects. The robot can only see markers which it is facing towards.

Each `Marker` object has the following attributes:

* `info`: a `MarkerInfo` object describing the marker itself. Has the following attributes:
  * `code`: the numeric code of the marker.
  * `marker_type`: the type of object the marker is attached to (either `MARKER_TOKEN_GOLD`, `MARKER_TOKEN_SILVER` or `MARKER_ARENA`).
  * `offset`: offset of the numeric code of the marker from the lowest numbered marker of its type. For example, token number 3 has the code 43, but offset 3.
  * `size`: the size that the marker would be in the real game, for compatibility with the SR API.
* `centre`: the location of the marker in polar coordinates, as a `PolarCoord` object. Has the following attributes:
  * `length`: the distance from the centre of the robot to the object (in metres).
  * `rot_y`: rotation about the Y axis in degrees.
* `dist`: an alias for `centre.length`
* `res`: the value of the `res` parameter of `R.see`, for compatibility with the SR API.
* `rot_y`: an alias for `centre.rot_y`
* `timestamp`: the time at which the marker was seen (when `R.see` was called).
	
**find_token():**
The robot can see all the the tokens around it in the map, in a field of view of 360 degrees and within a particular distance. This function checks all the tokens that the robot see thanks to the R.see() method and returns the distance and the angle between the robot and the closest silver token.
	
    Arguments
        None
    Returns
        dist: distance of the closest silver token (-1 if no silver token is detected)
	rot_y: angle between the robot and the silver token (-1 if no silver token is detected)
	marker: type of the token (Silver, Golden, Arena)
Code:
```python
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
```	

### The Grabber ###

The robot is equipped with a grabber, capable of picking up a token which is in front of the robot and within 0.4 metres of the robot's centre. To pick up a token, call the `R.grab` method:

```python
success = R.grab()
```

The `R.grab` function returns `True` if a token was successfully picked up, or `False` otherwise. If the robot is already holding a token, it will throw an `AlreadyHoldingSomethingException`.

To drop the token, call the `R.release` method.
		
code:
```python
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
```
### Collision Avoidance ###
to the collision with the golden tokens trail, I have first calculated the minimum distance to the nearest golden token in a circular sector and then applied the multiple conditions to drive the robot.
				
**Sector_min(dist, rot_y_min, rot_y_max):**
This function returns the distance of the nearest golden token respect the robot in the circular sector specified by the angles rot_y_min, rot_y_max and the radius equal to dist, moreover the try-except function is used to handle the case when there is no token in the circular sector.
Code:
```python
def Sector_min(dist, rot_y_min, rot_y_max):
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
```
**avoid_collision():**
In this function I have implemented multiple conditions to drive the robot around the arena without avoiding the golden token's boundary.
		
Code:		
```python
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
```

Main() Function
---------------------
In main() function I have called both functions (grab_silver_token(), avoid_collision()) in a while state so it can run continuesly.
