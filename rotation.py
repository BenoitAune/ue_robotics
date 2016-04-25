import pypot.robot
from pypot.robot import from_json
from contextlib import closing
import time
import numpy
from pypot.dynamixel import *
import json
import sys
import math


constL1 = 51
constL2 = 63.7
constL3 = 93
# Angle to match the theory with reality for theta 2 (measures of the triangle are 22.5, 60.7, 63.7). => Angle =  -20.69
theta2Correction = -20.69
# Same goes for theta 3 : +90 - 20.69 - a. Where a = asin(8.2/93) = 5.06
theta3Correction = 90 + theta2Correction - 5.06

# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c):
    value = ((a*a)+(b*b)-(c*c))/(2*a*b)
    #Note : to get the other altenative, simply change the sign of the return :
    return -math.acos(value)



def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2,l3=constL3) :
    theta1 = theta1 * math.pi / 180.0
    theta2 = (theta2 - theta2Correction) * math.pi / 180.0
    theta3 = -(theta3 - theta3Correction) * math.pi / 180.0

    planContribution = l1 + l2*math.cos(theta2) + l3*math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    return [x, y, z]



def computeIK(x, y, z, l1=constL1, l2=constL2,l3=constL3) :
    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x*x+y*y)-l1
    if (xp < 0) :
	print("Destination point too close")
	xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp,2) + math.pow(z,2))
    if (d > l2+l3):
	print("Destination point too far away")
	d = l2+l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    theta2 = alKashi(l2, d, l3) - math.atan2(z, xp)
    theta3 = math.pi - alKashi(l2, l3, d)

    return [modulo180(math.degrees(theta1)), modulo180(math.degrees(theta2) + theta2Correction), modulo180(math.degrees(theta3) + theta3Correction)]

#Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle) :
    if (-180 < angle < 180) :
	return angle

    angle  = angle % 360
    if (angle > 180) :
	return -360 + angle

    return angle

# Put all the legs to the initial position 
def posBase():

	for i in range (1,7):
		moveLeg(i,0,0,80)

	time.sleep(2)


# Take one leg number and make it move to put the center at the x,y,z position 
def moveLeg(a,x,y,z):

	# radius of the arbitrary virtual circle around the robot, the center of this circle is the center of the robot 
	R=100

	if a==1:

		position = computeIK(R - x, -y, -z,constL1,constL2,constL3)	

		my_robot.leg1[0].goal_position = position[0]
		my_robot.leg1[1].goal_position = position[1]
		my_robot.leg1[2].goal_position = position[2]	
	if a==2:

		
		position = computeIK(R * numpy.cos(numpy.pi / 4)+ y,R * numpy.sin(numpy.pi / 4) - x , -z ,constL1,constL2,constL3)

		my_robot.leg2[0].goal_position = position[0]
		my_robot.leg2[1].goal_position = position[1]
		my_robot.leg2[2].goal_position = position[2]

	if a==3:

		position = computeIK(R * numpy.cos(-numpy.pi / 4)+ y ,R * numpy.sin(-numpy.pi / 4)- x, -z,constL1,constL2,constL3)

		my_robot.leg3[0].goal_position = position[0]
		my_robot.leg3[1].goal_position = position[1]
		my_robot.leg3[2].goal_position = position[2]
	if a==4:

		position = computeIK(R +x, y , -z,constL1,constL2,constL3)

		my_robot.leg4[0].goal_position = position[0]
		my_robot.leg4[1].goal_position = position[1]
		my_robot.leg4[2].goal_position = position[2]

	if a==5:

		position = computeIK(R * numpy.cos(numpy.pi / 4)- y,R * numpy.sin(numpy.pi / 4) + x , -z,constL1,constL2,constL3)

		my_robot.leg5[0].goal_position = position[0]
		my_robot.leg5[1].goal_position = position[1]
		my_robot.leg5[2].goal_position = position[2]
	if a==6:

		position = computeIK(R * numpy.cos(-numpy.pi / 4)- y,R * numpy.sin(-numpy.pi / 4) + x, -z,constL1,constL2,constL3)

		my_robot.leg6[0].goal_position = position[0]
		my_robot.leg6[1].goal_position = position[1]
		my_robot.leg6[2].goal_position = position[2]


# demonstration of how we can move the center with all the legs staying on the floor 
def moveCenterDemo(slowness,sleep):
	
	dx=0
	dy=0
	dz=80
	i=0
	t=time.time()
	while(t+5 > time.time()):
	

		moveCenter(dx,dy,dz,sleep)

		i=i+(numpy.pi/slowness)
		dx=  25 *  numpy.cos(i) 
		dy=  25 *  numpy.sin(i) 
	
	t=time.time()
	while(t+5 > time.time()):
		
		
		moveCenter(0,0,dz,sleep)

		i=i+(numpy.pi/slowness)
		dz=  80 + 15 *  numpy.sin(i)
	moveCenter(0,0,80,0.02)



# Moving an arbitrary leg to an arbitrary (x, y, z) position
def moveOneLeg(a,x,y,z):

	R=100

	if a==1:

		position = computeIK(R + x, y, z,constL1,constL2,constL3)	

		my_robot.leg1[0].goal_position = position[0]
		my_robot.leg1[1].goal_position = position[1]
		my_robot.leg1[2].goal_position = position[2]	
	if a==2:

		
		position = computeIK(R + x, y, z,constL1,constL2,constL3)

		my_robot.leg2[0].goal_position = position[0]
		my_robot.leg2[1].goal_position = position[1]
		my_robot.leg2[2].goal_position = position[2]

	if a==3:

		position = computeIK(R + x, y, z,constL1,constL2,constL3)

		my_robot.leg3[0].goal_position = position[0]
		my_robot.leg3[1].goal_position = position[1]
		my_robot.leg3[2].goal_position = position[2]
	if a==4:

		position = computeIK(R + x, y, z,constL1,constL2,constL3)

		my_robot.leg4[0].goal_position = position[0]
		my_robot.leg4[1].goal_position = position[1]
		my_robot.leg4[2].goal_position = position[2]

	if a==5:

		position = computeIK(R + x, y, z,constL1,constL2,constL3)

		my_robot.leg5[0].goal_position = position[0]
		my_robot.leg5[1].goal_position = position[1]
		my_robot.leg5[2].goal_position = position[2]
	if a==6:

		position = computeIK(R + x, y, z,constL1,constL2,constL3)

		my_robot.leg6[0].goal_position = position[0]
		my_robot.leg6[1].goal_position = position[1]
		my_robot.leg6[2].goal_position = position[2]



# Take one leg number and make it move to the position that whe have to reach to rotate the center 
def moveRota(a,z,angleRad):


	R=100

	if a==1:

		position = computeIK(R * numpy.cos(angleRad), R * numpy.sin(angleRad) , z ,constL1,constL2,constL3)	

		my_robot.leg1[0].goal_position = position[0]
		my_robot.leg1[1].goal_position = position[1]
		my_robot.leg1[2].goal_position = position[2]	
	if a==2:

		
		position = computeIK(R * numpy.cos(angleRad + (numpy.pi / 4)), R * numpy.sin(angleRad + (numpy.pi / 4)) , z ,constL1,constL2,constL3)

		my_robot.leg2[0].goal_position = position[0]
		my_robot.leg2[1].goal_position = position[1]
		my_robot.leg2[2].goal_position = position[2]

	if a==3:

		position = computeIK(R * numpy.cos(angleRad - (numpy.pi / 4)), R * numpy.sin(angleRad - (numpy.pi / 4)), z ,constL1,constL2,constL3)

		my_robot.leg3[0].goal_position = position[0]
		my_robot.leg3[1].goal_position = position[1]
		my_robot.leg3[2].goal_position = position[2]
	if a==4:

		position = computeIK(R * numpy.cos(angleRad), R * numpy.sin(angleRad) , z ,constL1,constL2,constL3)

		my_robot.leg4[0].goal_position = position[0]
		my_robot.leg4[1].goal_position = position[1]
		my_robot.leg4[2].goal_position = position[2]

	if a==5:

		position = computeIK(R * numpy.cos(angleRad + (numpy.pi / 4)), R * numpy.sin(angleRad + (numpy.pi / 4)) , z ,constL1,constL2,constL3)

		my_robot.leg5[0].goal_position = position[0]
		my_robot.leg5[1].goal_position = position[1]
		my_robot.leg5[2].goal_position = position[2]
	if a==6:

		position = computeIK(R * numpy.cos(angleRad - (numpy.pi / 4)), R * numpy.sin(angleRad - (numpy.pi / 4)) , z ,constL1,constL2,constL3)

		my_robot.leg6[0].goal_position = position[0]
		my_robot.leg6[1].goal_position = position[1]
		my_robot.leg6[2].goal_position = position[2]


# Moving all the legs to rotate the center of an arbitrary angle 
def moveRotation(angleRad):
	moveRota(1,-80,angleRad)
	moveRota(2,-80,angleRad)
	moveRota(3,-80,angleRad)
	moveRota(4,-80,angleRad)
	moveRota(5,-80,angleRad)
	moveRota(6,-80,angleRad)	

# Moving the center of the robot to an arbitrary (x, y, z) position (the 6 legs staying on the floor)
def moveCenter(x,y,z,sleep):
	for i in range (1,7):
		moveLeg(i,x,y,z)
	time.sleep(sleep)



# Rotating the robot to an arbitrary angle (odometry)   
def rotation(angle) :
	
	#ne marche que pour les angles positifs :/
	angleRad=math.radians(angle)

	if (angleRad<(numpy.pi/6)):
		moveRotation(angleRad) # Rotate the center 
		# Moving all the legs one by one to the initial position 		
		time.sleep(0.3)
		moveLeg(1,0,0,60)
		time.sleep(0.1)
		moveLeg(1,0,0,80)
		time.sleep(0.1)
		moveLeg(2,0,0,60)
		time.sleep(0.1)
		moveLeg(2,0,0,80)
		time.sleep(0.1)
		moveLeg(3,0,0,60)
		time.sleep(0.1)
		moveLeg(3,0,0,80)
		time.sleep(0.1)
		moveLeg(4,0,0,60)
		time.sleep(0.1)
		moveLeg(4,0,0,80)
		time.sleep(0.1)
		moveLeg(5,0,0,60)
		time.sleep(0.1)
		moveLeg(5,0,0,80)
		time.sleep(0.1)
		moveLeg(6,0,0,60)
		time.sleep(0.1)
		moveLeg(6,0,0,80)
		time.sleep(0.3)

	# Do the same operation until the angle is reached 	
	else :  
		nbrRotaInt = angleRad / (numpy.pi / 6);
		reste = angleRad % (numpy.pi/6);

		for i in range(int(nbrRotaInt)):
			moveRotation(numpy.pi/6)
			time.sleep(0.3)
			moveLeg(1,0,0,60)
			time.sleep(0.1)
			moveLeg(1,0,0,80)
			time.sleep(0.1)
			moveLeg(2,0,0,60)
			time.sleep(0.1)
			moveLeg(2,0,0,80)
			time.sleep(0.1)
			moveLeg(3,0,0,60)
			time.sleep(0.1)
			moveLeg(3,0,0,80)
			time.sleep(0.1)
			moveLeg(4,0,0,60)
			time.sleep(0.1)
			moveLeg(4,0,0,80)
			time.sleep(0.1)
			moveLeg(5,0,0,60)
			time.sleep(0.1)
			moveLeg(5,0,0,80)
			time.sleep(0.1)
			moveLeg(6,0,0,60)
			time.sleep(0.1)
			moveLeg(6,0,0,80)
			time.sleep(0.3)

		if ( reste > (numpy.pi/300) ):
			moveRotation(reste)
			time.sleep(0.3)
			moveLeg(1,0,0,60)
			time.sleep(0.1)
			moveLeg(1,0,0,80)
			time.sleep(0.1)
			moveLeg(2,0,0,60)
			time.sleep(0.1)
			moveLeg(2,0,0,80)
			time.sleep(0.1)
			moveLeg(3,0,0,60)
			time.sleep(0.1)
			moveLeg(3,0,0,80)
			time.sleep(0.1)
			moveLeg(4,0,0,60)
			time.sleep(0.1)
			moveLeg(4,0,0,80)
			time.sleep(0.1)
			moveLeg(5,0,0,60)
			time.sleep(0.1)
			moveLeg(5,0,0,80)
			time.sleep(0.1)
			moveLeg(6,0,0,60)
			time.sleep(0.1)
			moveLeg(6,0,0,80)
			time.sleep(0.3)

# Instruction to do one step
def moveLegWalk(x, y, z,sleep) :
	

	moveLeg(1,x,y,z);
	moveLeg(2,x,y,z);
	moveLeg(3,x,y,z);
	moveLeg(4,x,y,z);
	moveLeg(5,x,y,z);
	moveLeg(6,x,y,z);

	
	time.sleep(sleep);

	moveLeg(1, x, y, z - 30);
	moveLeg(3, x, y, z - 30);
	moveLeg(5, x, y, z - 30);
	
	time.sleep(sleep);
	
	moveLeg(1, 0, 0, z - 30);
	moveLeg(3, 0, 0, z - 30);
	moveLeg(5, 0, 0, z - 30);
	
	time.sleep(sleep);
	
	moveLeg(1, 0, 0, z);
	moveLeg(3, 0, 0, z);
	moveLeg(5, 0, 0, z);

	time.sleep(sleep);

	moveLeg(2, x, y, z - 30);
	moveLeg(4, x, y, z - 30);
	moveLeg(6, x, y, z - 30);

	time.sleep(sleep);

	moveLeg(2, 0, 0, z - 30);
	moveLeg(4, 0, 0, z - 30);
	moveLeg(6, 0, 0, z - 30);

	time.sleep(sleep);

	moveLeg(2, 0, 0, z);
	moveLeg(4, 0, 0, z);
	moveLeg(6, 0, 0, z);

	time.sleep(sleep);


# Do all the step to reach the arbitrary x,y,z position given 
def walk(x, y, z,sleep):
	p = 30;
	if (x < p and y < p) :
		moveLegWalk(x, y, z,sleep);
	else :
		n = max(x , y) / p;
		for i in range(int(n)) :
			moveLegWalk(x/n, y/n, z,sleep);
		if ((x % p) != 0 or (y % p) != 0) :
			moveLegWalk((x % p), (y % p), z,sleep);


# demonstration 
if __name__ == '__main__' :
	
	with closing(from_json('BB-9.json')) as my_robot :
		
		for m in my_robot.motors :
			m.compliant = False
		
		posBase()
		
		#moveOneLeg(5,0,0,0)
		#time.sleep(0.5)
		#moveOneLeg(5,40,40,80)
		#time.sleep(1)
		#moveCenterDemo(48,0.01)
		#time.sleep(0.5)
		#moveCenterDemo(800,0.001)
			
		#walk(300,-200,80,0.1)
		
		rotation(90)
