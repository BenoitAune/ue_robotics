import time
from pypot.robot import from_json
from contextlib import closing




with closing(from_json('robotConfig.json')) as my_robot:
    for m in my_robot.leg1:
        m.compliant = False
	m.goal_position = 0.0
    for m in my_robot.leg3:
        m.compliant = False
	m.goal_position = 0.0
    for m in my_robot.leg5:
        m.compliant = False
	m.goal_position = 0.0

    for m in my_robot.leg2:
        m.compliant = False
	m.goal_position = -45.0
    for m in my_robot.leg4:
        m.compliant = False
	m.goal_position = -45.0
    for m in my_robot.leg6:
        m.compliant = False
	m.goal_position = -45.0
	    	

