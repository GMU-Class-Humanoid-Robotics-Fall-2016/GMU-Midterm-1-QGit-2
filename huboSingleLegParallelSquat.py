#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */



import hubo_ach as ha
import ach
import sys
import time
from ctypes import *

import math


LOWER_LEG_D1 = 300.38
UPPER_LEG_D2 = 300.03

ANKLE_LENGTH = 94.97
HIP_TO_WAIST_LENGTH = 289.47 - 107.0

#Part 1
PART_1_LEFT_DOWN = 210.0
PART_1_RIGHT_DOWN = 200.0 

PART_1_BEND_LEFT_LEG_Y = LOWER_LEG_D1 + UPPER_LEG_D2 - PART_1_LEFT_DOWN
PART_1_BEND_LEFT_LEG_X = 0.0

PART_1_BEND_RIGHT_LEG_Y = LOWER_LEG_D1 + UPPER_LEG_D2 - PART_1_RIGHT_DOWN
PART_1_BEND_RIGHT_LEG_X = 0.0


#Part 2
PART_2_LEFT_DOWN = 100.0
PART_2_RIGHT_DOWN = 110.0

PART_2_BEND_LEFT_LEG_Y = LOWER_LEG_D1 + UPPER_LEG_D2 - PART_2_LEFT_DOWN
PART_2_BEND_LEFT_LEG_X = 0.0

PART_2_BEND_RIGHT_LEG_Y = LOWER_LEG_D1 + UPPER_LEG_D2 - PART_2_RIGHT_DOWN
PART_2_BEND_RIGHT_LEG_X = 0.0

PARALLEL_THETA = -.85 # Hip joing can exceed ninety degrees , this is the most it can lean forward, when trying to go down .1 m

LEAN_ANGLE = 0.12 #0.11318


def inverseKinematics(x,y,d1,d2):
	theta2 = math.acos((x*x+y*y-d1*d1-d2*d2)/(2.0*d1*d2))
	theta1 = math.atan2(y*(d1+d2*math.cos(theta2)) - x*d2*math.sin(theta2) ,  x*(d1+d2*math.cos(theta2)) + y*d2*math.sin(theta2) )

	theta1 = -1.0*(math.pi/2.0 - theta1)

	print "Theta1 (RAP,LAP,RHP,LHP) = ", theta1
	print "Theta2 (RKN,LKN)         = ", theta2

	return (theta1, theta2)

def part1LeanRight(ref, r, s, state):
	
	step = 0.0
	step_size = 0.02

	while(step < LEAN_ANGLE):
		ref.ref[ha.RAR] = -step 
		ref.ref[ha.LAR] = -step 

		ref.ref[ha.RHR] = step 
		ref.ref[ha.LHR] = step 
	
		r.put(ref)
		
		step = step + step_size
		
		simSleep(.1, s, state ) #time.sleep(0.1)

		print "Lean Right : Lean Angle = ", LEAN_ANGLE, "Current Angle = ", step
	
	ref.ref[ha.RAR] = -LEAN_ANGLE
	ref.ref[ha.LAR] = -LEAN_ANGLE

	ref.ref[ha.RHR] = LEAN_ANGLE
	ref.ref[ha.LHR] = LEAN_ANGLE	

	r.put(ref)

	simSleep(.5, s, state ) 
	
	return

def part2LeanLeft(ref, r, s, state):
	
	step = 0.0
	step_size = 0.02

	while(step < LEAN_ANGLE):
		ref.ref[ha.RAR] = step 
		ref.ref[ha.LAR] = step 

		ref.ref[ha.RHR] = -step 
		ref.ref[ha.LHR] = -step 
	
		r.put(ref)
		
		step = step + step_size
		
		simSleep(.1, s, state ) #time.sleep(0.1)

		print "Lean Left : Lean Angle = ", LEAN_ANGLE, "Current Angle = ", step
	
	ref.ref[ha.RAR] = LEAN_ANGLE
	ref.ref[ha.LAR] = LEAN_ANGLE

	ref.ref[ha.RHR] = -LEAN_ANGLE
	ref.ref[ha.LHR] = -LEAN_ANGLE	
	
	r.put(ref)

	simSleep(.1, s, state ) 
	
	return
	

def part1BendLeftLeg(ref, r, s, state):

	theta1, theta2 = inverseKinematics(PART_1_BEND_LEFT_LEG_X, PART_1_BEND_LEFT_LEG_Y, LOWER_LEG_D1, UPPER_LEG_D2)
	
	ref.ref[ha.LKN] = theta2

	ref.ref[ha.LHP] = theta1

	ref.ref[ha.LAP] = theta1 

	# Write to the feed-forward channel
	r.put(ref)
	
	print "Bend Left : LKN = ", theta2, " LHP,LAP = ", theta1
	simSleep(0.5, s, state) #time.sleep(5.0)
	
	return

def part2BendRightLeg(ref, r, s, state):

	theta1, theta2 = inverseKinematics(PART_2_BEND_RIGHT_LEG_X, PART_2_BEND_RIGHT_LEG_Y, LOWER_LEG_D1, UPPER_LEG_D2)
	
	ref.ref[ha.RKN] = theta2

	ref.ref[ha.RHP] = theta1

	ref.ref[ha.RAP] = theta1 

	# Write to the feed-forward channel
	r.put(ref)
	
	print "Bend Right : RKN = ", theta2, " RHP,RAP = ", theta1
	simSleep(0.5, s, state) #time.sleep(5.0)
	
	return

def part1ReturnCenter(ref, r, s, state):

	start_theta1, start_theta2 = inverseKinematics(PART_1_BEND_LEFT_LEG_X, PART_1_BEND_LEFT_LEG_Y, LOWER_LEG_D1, UPPER_LEG_D2)
	
	step_theta1 = start_theta1;
	step_size = 0.2;

	step_theta2 = start_theta2;
	
	while(step_theta2 > 0.0):	
	
		ref.ref[ha.LKN] = step_theta2
		ref.ref[ha.LHP] = step_theta1
		ref.ref[ha.LAP] = step_theta1 

		# Write to the feed-forward channel
		r.put(ref)

		simSleep(0.05, s, state) #time.sleep(4.0)

		step_theta1 = step_theta1 + step_size
		step_theta2 = step_theta2 - 2.0*step_size
	
	ref.ref[ha.LKN] = 0.0
	ref.ref[ha.LHP] = 0.0
	ref.ref[ha.LAP] = 0.0
	
	r.put(ref)
	simSleep(0.05, s, state)

	#return center
	step = LEAN_ANGLE
	step_size = 0.02

	while(step > 0.0):
		ref.ref[ha.RAR] = -step 
		ref.ref[ha.LAR] = -step 

		ref.ref[ha.RHR] = step 
		ref.ref[ha.LHR] = step 
	
		r.put(ref)
		
		step = step - step_size
		
		simSleep(.05, s, state ) #time.sleep(0.1)

		print "Lean Right : Lean Angle = ", LEAN_ANGLE, "Current Angle = ", step
	
	ref.ref[ha.RAR] = 0.0
	ref.ref[ha.LAR] = 0.0

	ref.ref[ha.RHR] = 0.0
	ref.ref[ha.LHR] = 0.0	

	r.put(ref)

def part2ReturnCenter(ref, r, s, state):

	#simSleep(0.1, s, state) #time.sleep(4.0)

	start_theta1, start_theta2 = inverseKinematics(PART_2_BEND_RIGHT_LEG_X, PART_2_BEND_RIGHT_LEG_Y, LOWER_LEG_D1, UPPER_LEG_D2)
	
	step_theta1 = start_theta1;
	step_size = 0.1;

	step_theta2 = start_theta2;
	
	while(step_theta2 > 0.0):	
	
		ref.ref[ha.RKN] = step_theta2
		ref.ref[ha.RHP] = step_theta1
		ref.ref[ha.RAP] = step_theta1 

		# Write to the feed-forward channel
		r.put(ref)

		simSleep(0.05, s, state) #time.sleep(4.0)

		step_theta1 = step_theta1 + step_size
		step_theta2 = step_theta2 - 2.0*step_size
	
	ref.ref[ha.LKN] = 0.0
	ref.ref[ha.LHP] = 0.0
	ref.ref[ha.LAP] = 0.0
	
	r.put(ref)
	simSleep(0.05, s, state)

	#return center
	step = LEAN_ANGLE
	step_size = 0.02

	while(step > 0.0):
		ref.ref[ha.RAR] = step 
		ref.ref[ha.LAR] = step 

		ref.ref[ha.RHR] = -step 
		ref.ref[ha.LHR] = -step 
	
		r.put(ref)
		
		step = step - step_size
		
		simSleep(.05, s, state ) #time.sleep(0.1)

		print "Lean Right : Lean Angle = ", LEAN_ANGLE, "Current Angle = ", step
	
	ref.ref[ha.RAR] = 0.0
	ref.ref[ha.LAR] = 0.0

	ref.ref[ha.RHR] = 0.0
	ref.ref[ha.LHR] = 0.0	

	r.put(ref)

def part1SquatRightLeg(ref, r, s, state):
	
	print "Right Leg X = ", PART_1_BEND_RIGHT_LEG_X
	print "Right Leg Y = ", PART_1_BEND_RIGHT_LEG_Y	

	theta1, theta2 = inverseKinematics(PART_1_BEND_RIGHT_LEG_X, PART_1_BEND_RIGHT_LEG_Y, LOWER_LEG_D1, UPPER_LEG_D2)
	
	step_theta1 = 0.0
	step_size_theta1 = 0.05

	step_theta2 = 0.0
	step_size_theta2 = 0.1
	
	for i in range(2):
		# DOWN	
		print "Down"
		while(step_theta2 < theta2):
	
			ref.ref[ha.RKN] = step_theta2
			ref.ref[ha.RHP] = -step_theta1
			ref.ref[ha.RAP] = -step_theta1 
		
			r.put(ref)
		
			step_theta1 = step_size_theta1 + step_theta1
			step_theta2 = step_size_theta2 + step_theta2
		
			simSleep(.1, s, state) #time.sleep(0.2)
			print "Step_theta1 = ", step_theta1
	
		ref.ref[ha.RKN] = theta2
		ref.ref[ha.RHP] = theta1
		ref.ref[ha.RAP] = theta1

		# Write to the feed-forward channel
		r.put(ref)
		
		simSleep(.1, s, state)

		step_theta1 = step_theta1 - step_size_theta1
		step_theta2 = step_theta2 - step_size_theta2
		

		#UP
		print "UP"
		while(step_theta2 > 0.0):
			print "Step_theta1 = ", step_theta1
			ref.ref[ha.RKN] = step_theta2
			ref.ref[ha.RHP] = -step_theta1
			ref.ref[ha.RAP] = -step_theta1 
		
			r.put(ref)
		
			step_theta1 = step_theta1 - step_size_theta1
			step_theta2 = step_theta2 - step_size_theta2
		
			simSleep(.1, s, state) #time.sleep(0.2)
	return

def part2SquatLeftLeg(ref, r, s, state):
	
	print "Left Leg X = ", PART_2_BEND_RIGHT_LEG_X
	print "Left Leg Y = ", PART_2_BEND_RIGHT_LEG_Y	

	theta1, theta2 = inverseKinematics(PART_2_BEND_LEFT_LEG_X, PART_2_BEND_LEFT_LEG_Y, LOWER_LEG_D1, UPPER_LEG_D2)
	
	step_theta1 = 0.0
	step_size_theta1 = 0.05

	step_theta2 = 0.0
	step_size_theta2 = 0.1
	
	for i in range(2):
		# DOWN	
		while(step_theta2 < theta2):
	
			ref.ref[ha.LKN] = step_theta2
			ref.ref[ha.LHP] = step_theta1 + PARALLEL_THETA
			ref.ref[ha.LAP] = step_theta1 
		
			r.put(ref)
		
			step_theta1 = step_theta1 - step_size_theta1 
			step_theta2 = step_theta2 + step_size_theta2 
		
			simSleep(.1, s, state) #time.sleep(0.2)
			print "Step_theta1 = ", step_theta1
	
		ref.ref[ha.LKN] = theta2
		ref.ref[ha.LHP] = theta1 + PARALLEL_THETA
		ref.ref[ha.LAP] = theta1

		# Write to the feed-forward channel
		r.put(ref)

		simSleep(.1, s, state)

		step_theta1 = step_theta1 + step_size_theta1
		step_theta2 = step_theta2 - step_size_theta2

		#UP
		print "UP"
		while(step_theta2 > 0.0):
			print "Step_theta1 = ", step_theta1
			ref.ref[ha.LKN] = step_theta2
			ref.ref[ha.LHP] = step_theta1 + PARALLEL_THETA
			ref.ref[ha.LAP] = step_theta1 
		
			r.put(ref)
		
			step_theta1 = step_theta1 + step_size_theta1
			step_theta2 = step_theta2 - step_size_theta2
		
			simSleep(.1, s, state) #time.sleep(0.2)
	return
def part2Parallel(ref, r, s, state):
	
	step_theta = 0.0
	step_size_theta = 0.05
	
	while(step_theta > PARALLEL_THETA):

		ref.ref[ha.LHP] = step_theta
	
		r.put(ref)
	
		step_theta = step_theta - step_size_theta
	
		simSleep(.1, s, state) 

		print "Parallel_Step_theta = ", step_theta

	simSleep(.5, s, state) 
	
	part2SquatLeftLeg(ref, r, s, state)

	step_theta = step_theta + 2*step_size_theta
	
	simSleep(.5, s, state) 	
	
	while(step_theta < 0.0):

		ref.ref[ha.LHP] = step_theta
	
		r.put(ref)
	
		step_theta = step_theta + step_size_theta
	
		simSleep(.1, s, state) 

		print "Parallel_Step_theta = ", step_theta
	
	simSleep(.5, s, state) 
	return


def simSleep(sec, s, state):
	tick = state.time;
	dt = 0;
	while(dt <= sec):
		s.get(state, wait=False, last=True)
		dt = state.time - tick;
	return




# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
#r = ach.Channel("huboFilterChan")
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=True)
	

#todo
part1LeanRight(ref,r, s, state)

part1BendLeftLeg(ref, r, s, state)

part1SquatRightLeg(ref, r, s, state)

part1ReturnCenter(ref,r, s, state)

simSleep(2.0, s, state)

part2LeanLeft(ref,r,s,state)

part2BendRightLeg(ref, r, s, state)

part2Parallel(ref, r, s, state)
#part2SquatLeftLeg(ref, r, s, state)

part2ReturnCenter(ref, r, s, state)

print "Done"

# Close the connection to the channels
r.close()
s.close()





	

