import pypot.robot
from pypot.robot import from_json
from contextlib import closing
import time
import numpy
from pypot.dynamixel import *
import json
import sys









#with closing(from_json('BB-9.json')) as my_robot :
my_robot=from_json('BB-9.json')

for m in my_robot.motors :
	m.compliant = False	
	
angle = 20
rectificationAngleDeBase = 70
positionMotorThird = 0



while (True) :
	positionMotorSecond = min(angle * numpy.sin(time.time()), 0)
	positionMotorSecondBis = min(angle * numpy.sin(time.time() + numpy.pi), 0)

	#---------------------------Groupe 1 de leg------------------------------#
	my_robot.leg1[0].goal_position = angle * numpy.cos(time.time() + numpy.pi)
	my_robot.leg3[0].goal_position = angle * numpy.cos(time.time()) - rectificationAngleDeBase
	my_robot.leg5[0].goal_position = angle * numpy.cos(time.time()) + rectificationAngleDeBase
	
	my_robot.leg1[1].goal_position = positionMotorSecond
	my_robot.leg3[1].goal_position = positionMotorSecond
	my_robot.leg5[1].goal_position = positionMotorSecond

	my_robot.leg1[2].goal_position = positionMotorThird
	my_robot.leg3[2].goal_position = positionMotorThird
	my_robot.leg5[2].goal_position = positionMotorThird	

	#---------------------------Groupe 2 de leg------------------------------#
	my_robot.leg2[0].goal_position = angle * numpy.cos(time.time()) + rectificationAngleDeBase
	my_robot.leg4[0].goal_position = angle * numpy.sin(time.time())
	my_robot.leg6[0].goal_position = angle * numpy.cos(time.time()) - rectificationAngleDeBase
	
	my_robot.leg2[1].goal_position = positionMotorSecondBis
	my_robot.leg4[1].goal_position = positionMotorSecondBis
	my_robot.leg6[1].goal_position = positionMotorSecondBis

	my_robot.leg2[2].goal_position = positionMotorThird
	my_robot.leg4[2].goal_position = positionMotorThird
	my_robot.leg6[2].goal_position = positionMotorThird

	time.sleep(0.001)
			
