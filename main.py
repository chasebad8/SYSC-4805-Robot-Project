import math
import time
from enum import Enum

import sensorDistance

"""
SYSC 4805 Snow Plowing Robot
Group 7 (Lizard Green)
"""

# Try to connect to coppeliasim python API library
try:
    import sim
except ImportError:
    print('--------------------------------------------------------------')
    print('Make sure the right files are in this folder')
    print('--------------------------------------------------------------')
    print('')
    raise  # Kill the program after

import time as t
import math
import random

from sim import simxReadVisionSensor

global clientID

MAX_SPEED = 2 #m/s
MED_SPEED = 1
SLOW_SPEED = 0.5

RADIUS = 0.043 #m

#if the direction changed
top_dir_chnage = False

#the euler angles for x
directionAngles = [0, 1.5128347873687744, 0, -1.5128347873687744]

#An enumeration that corresponds to cardinal points
class Orientation(Enum):
    north = 0
    east  = 1
    south = 2
    west  = 3

#This class holds all information about the robots heading
class Heading():
    def __init__(self):
        #The current heading of the robot
        self.curr_heading = 0

        #The previous heading of the robot
        self.prev_heading = 0

        self.left_sensor_prev = 0
        self.right_sensor_prev = 0
        self.obj_count = 0

    #Increment the number of objects detected in a row
    def inc_obj_count(self):
        self.obj_count += 1

    def get_obj_count(self):
        return self.obj_count

    #reset obj count back to 0
    def reset_obj_count(self):
        self.obj_count = 0

    def check_obj_count(self):
        if self.obj_count >= 2:
            return True
        else:
            return False

    #update the current heading to a left turn
    def update_left(self):
        self.prev_heading = self.curr_heading
        self.curr_heading += 3

    #update the current heading to a right turn
    def update_right(self):
        self.prev_heading = self.curr_heading
        self.curr_heading += 1

    #update the current heading to a 180 degree u-turn
    def update_uturn(self):
        self.prev_heading = self.curr_heading
        self.curr_heading += 2

    def get_curr_heading(self):
        return (self.curr_heading % 4)

    def get_prev_heading(self):
        return (self.prev_heading % 4)

    def get_prev_heading_enum(self):
        return Orientation(self.prev_heading % 4)

    def get_curr_heading_enum(self):
        return Orientation(self.curr_heading % 4)

    def update_left_prev(self, left_prev):
        self.left_sensor_prev = left_prev

    def update_right_prev(self, right_prev):
        self.right_sensor_prev = right_prev

    def get_left_prev(self):
        return self.left_sensor_prev

    def get_right_prev(self):
        return self.right_sensor_prev

'''
setWheelVelocity
-----------------------------------------------------------------
Set the a wheels velocity is rad/s. 

param:
    handle:   wheel object ID
    velocity: The velocity the wheel should be set at (in rad/s)

return: 
'''
def setWheelVelocity(handle, velocity):
    return sim.simxSetJointTargetVelocity(clientID, handle, velocity, sim.simx_opmode_oneshot)

'''
getObjectHandle
-----------------------------------------------------------------
Return the objectID of an object by passing in the sim name

param:
    obj_name: the name of the object in Coppelia scene

return: a simulations object's object ID
'''
def getObjectHandle(obj_name):
    return sim.simxGetObjectHandle(clientID, obj_name, sim.simx_opmode_blocking)

'''
move_plow_handle
-----------------------------------------------------------------
Move the scooper up or down

param:
    plow_ID_left:  ID of the left plow scooper joint 
    plow_ID_right: ID of the right plow scooper joint 
    position:

return: None
'''
def move_plow_handle(plow_ID_left, plow_ID_right, position):
    if(position == "UP"):
        _ = sim.simxSetJointTargetPosition(clientID, plow_ID_left, 0, sim.simx_opmode_oneshot)
        _ = sim.simxSetJointTargetPosition(clientID, plow_ID_right, 0, sim.simx_opmode_oneshot)

    elif (position == "DOWN"):
        _ = sim.simxSetJointTargetPosition(clientID, plow_ID_left, math.pi/2, sim.simx_opmode_oneshot)
        _ = sim.simxSetJointTargetPosition(clientID, plow_ID_right, -math.pi/2, sim.simx_opmode_oneshot)

'''
init_plow
-----------------------------------------------------------------
initialize the plow by opening up the plow arms

param:
    plow_ID_left:  ID of the left plow opening joint 
    plow_ID_right: ID of the right plow opening joint

return: None
'''
def init_plow(plow_ID_left, plow_ID_right):
        _ = sim.simxSetJointTargetPosition(clientID, plow_ID_left, -math.pi/2, sim.simx_opmode_oneshot)
        _ = sim.simxSetJointTargetPosition(clientID, plow_ID_right, -math.pi/2, sim.simx_opmode_oneshot)

'''
startSimulation
-----------------------------------------------------------------
Connect to the CoppeliaSim simulator

param: None

return: the Client ID for the sim
'''
def startSimulation():
    print('Program started')
    sim.simxFinish(-1)
    return sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

'''
turn
-----------------------------------------------------------------
Execute a 90 degree turn to the right or left with 
respect to current heading

param:
    turnRight:   Boolean where True indicated right 90 
                 degree turn False for left 90 degree turn
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    robot:       Robot ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)

return: True for successful turn, False for a failed turn (timeout)
'''
def turn(turnRight, left_joint, right_joint, robot, heading):
    startTime=time.time()
    _, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    targetAngle = directionAngles[heading]
    speed = 2

    #If a right turn is requested turn to the right
    if turnRight:
        _ = setWheelVelocity(left_joint, speed/RADIUS)
        _ = setWheelVelocity(right_joint, -speed/RADIUS)

    elif not turnRight:
        _ = setWheelVelocity(left_joint, -speed/RADIUS)
        _ = setWheelVelocity(right_joint, speed/RADIUS)

    #rotate until the target cardinal angle is only off by +-0.01 radians
    while True:
        _, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
        if abs(orientation[1] - targetAngle) <= 0.01:
            _ = setWheelVelocity(left_joint,0)
            _ = setWheelVelocity(right_joint,0)
            break

        #Divide the speed as the rotation gets closer to be more accurate
        #but once its 0.025 do not let it get any slower
        speed = 0.025 if (speed / 1.25) < 0.025 else (speed/1.25) 
        t.sleep(0.025)

        #set the new slower speed
        if turnRight:
            _ = setWheelVelocity(left_joint, speed/RADIUS)
            _ = setWheelVelocity(right_joint, -speed/RADIUS)

        elif not turnRight:
            _ = setWheelVelocity(left_joint, -speed/RADIUS)
            _ = setWheelVelocity(right_joint, speed/RADIUS)

        #If it takes longer than 15 seconds to reach the angle it wants, timeout
        if time.time() - startTime > 15:
            print("Timeout Occured")
            _ = setWheelVelocity(left_joint,0)
            _ = setWheelVelocity(right_joint,0)
            return False

    print("facing " + str(Orientation(heading))) 
    return True

'''
u_turn
-----------------------------------------------------------------
Execute a 180 degree turn with respect to current heading

param:
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    robot:       Robot ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)
    
return: True for successful turn, False for a failed turn (timeout)
'''
def u_turn(left_joint, right_joint, robot, heading):
    startTime=time.time()
    _, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    targetAngle = directionAngles[heading]
    speed = 2

    _ = setWheelVelocity(left_joint, speed/RADIUS)
    _ = setWheelVelocity(right_joint, -speed/RADIUS)
    t.sleep(0.225)

    while True:
        _, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
        if abs(orientation[1] - targetAngle) <= 0.01:
            _ = setWheelVelocity(left_joint,0)
            _ = setWheelVelocity(right_joint,0)            
            break

        speed = 0.025 if (speed / 1.25) < 0.025 else (speed/1.25) 
        t.sleep(0.025)

        _ = setWheelVelocity(left_joint, speed/RADIUS)
        _ = setWheelVelocity(right_joint, -speed/RADIUS)

        if abs(orientation[1] - targetAngle) <= 0.01:
            break

        if time.time() - startTime > 15:
            print("Timeout Occured")
            return False
            _ = setWheelVelocity(left_joint,0)
            _ = setWheelVelocity(right_joint,0)

    print("facing " + str(Orientation(heading))) 

    return True

'''
long_lat_turn
-----------------------------------------------------------------
If going north or south turn east or west respectively

long ( -- ) lat ( | )

param:
    turnRight:   Boolean where True indicated right turn False for left turn
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    remote_bot:  Robot ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)
    
return: 
'''
def long_lat_turn(turnRight, left_joint, right_joint, remote_bot, heading):
    if(turnRight):
        heading.update_right()
        turn(True, left_joint, right_joint, remote_bot, heading.get_curr_heading())
    else:
        heading.update_left()
        turn(False, left_joint, right_joint, remote_bot, heading.get_curr_heading())            

'''
lat_long_lat_turn
-----------------------------------------------------------------
If current heading is east: turn north -> forward 1m -> turn west
If current heading is west: turn north -> forward 1m -> turn east

param:
    turnRight:   Boolean where True indicated right turn False for left turn
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    remote_bot:  Robot ID
    scoop_left:  The left scoop joint ID
    scoop_right: The right scoop joint ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)
    
return: None
'''
def lat_long_lat_turn(turnRight, left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading):
    if turnRight:
        heading.update_right()
        turn(True, left_joint, right_joint, remote_bot, heading.get_curr_heading())
    else:
        heading.update_left()
        turn(False, left_joint, right_joint, remote_bot, heading.get_curr_heading())

    #If there is an object in the way while the robot wants to move forward, hug the wall to get around it
    _ = sim.simxGetObjectGroupData(clientID, 5, 13, sim.simx_opmode_blocking)
    if sim.simxReadProximitySensor(clientID, front_prox_sensor, sim.simx_opmode_streaming)[1]:
        print("Object avoidance behaviour: HUG WALL")
        hug_wall(left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading, 
                 left_prox_sensor, right_prox_sensor, front_prox_sensor)
    
    #Move forward 1m
    t.sleep(0.25)
    _ = setWheelVelocity(left_joint, MED_SPEED/RADIUS)
    _ = setWheelVelocity(right_joint, MED_SPEED/RADIUS)
    t.sleep(0.75)
    _ = setWheelVelocity(left_joint, 0/RADIUS)
    _ = setWheelVelocity(right_joint, 0/RADIUS)
    t.sleep(0.25)

    #then turn again depending on requested turn direction
    if turnRight:
        heading.update_right()
        turn(True, left_joint, right_joint, remote_bot, heading.get_curr_heading())
    else:
        heading.update_left()
        turn(False, left_joint, right_joint, remote_bot, heading.get_curr_heading())

'''
line_detected
-----------------------------------------------------------------
If a line is detected handle it based off of what direction it was hit

param:
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    remote_bot:  Robot ID
    scoop_left:  The left scoop joint ID
    scoop_right: The right scoop joint ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)
    
return: 
'''
def line_detected(left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading):

    #If you hit a line, back up
    _ = setWheelVelocity(left_joint, -MED_SPEED/RADIUS)
    _ = setWheelVelocity(right_joint, -MED_SPEED/RADIUS)
    t.sleep(1) 
    _ = setWheelVelocity(left_joint, 0)
    _ = setWheelVelocity(right_joint, 0)
    t.sleep(0.5)

    #If currently facing north then turn right
    if heading.get_curr_heading_enum() == Orientation.north:
        print("North to East")
        long_lat_turn(True, left_joint, right_joint, remote_bot, heading)

    #If currently facing east, then do a 180 while going north 1m
    elif heading.get_curr_heading_enum() == Orientation.east:
        print("East to North to West")
        lat_long_lat_turn(False , left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading)

    #If currently facing south then turn right
    elif heading.get_curr_heading_enum() == Orientation.south:
        print("South to East")
        long_lat_turn(False, left_joint, right_joint, remote_bot, heading)


    #If currently facing west, then do a 180 while going north 1m
    elif heading.get_curr_heading_enum() == Orientation.west:
        print("West to North to East")
        lat_long_lat_turn(True, left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading)

'''
hug_wall
-----------------------------------------------------------------
Hug the wall so that a loop does not occur

param:
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    remote_bot:  Robot ID
    scoop_left:  The left scoop joint ID
    scoop_right: The right scoop joint ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)
    
return: 
'''
def hug_wall(left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading, 
    left_prox_sensor, right_prox_sensor, front_prox_sensor):

        watchdog = t.time()

        curr_f = sensorDistance.flat_distance(clientID, front_prox_sensor)
        turns = 0

        #get 0.9m away from the wall
        test = sensorDistance.flat_distance(clientID, front_prox_sensor)
        while(test > 0.9):
                test = sensorDistance.flat_distance(clientID, front_prox_sensor)
                _=setWheelVelocity(left_joint, MED_SPEED/RADIUS)
                _=setWheelVelocity(right_joint, MED_SPEED/RADIUS)

        # stopping and dropping scoop
        _=setWheelVelocity(left_joint, 0)
        _=setWheelVelocity(right_joint, 0)
        _=move_plow_handle(scoop_left, scoop_right, "DOWN")
        t.sleep(1)

        curr_l = sensorDistance.flat_distance(clientID, left_prox_sensor)
        curr_r = sensorDistance.flat_distance(clientID, right_prox_sensor)
        curr_f = sensorDistance.flat_distance(clientID, front_prox_sensor)

        #If currently facing north then turn right
        if heading.get_curr_heading_enum() == Orientation.north and heading.get_prev_heading_enum() == Orientation.west:
            print("Hug wall turn right")

            #turn right
            long_lat_turn(True, left_joint, right_joint, remote_bot, heading)
            _=setWheelVelocity(left_joint, 0)
            _=setWheelVelocity(right_joint, 0)
            t.sleep(0.5)

            #increment the number of turns used
            turns += 1

            obst = 1

            #drop the plow and move forward
            _=move_plow_handle(scoop_left, scoop_right, "UP")
            _=setWheelVelocity(left_joint, MED_SPEED/RADIUS)
            _=setWheelVelocity(right_joint, MED_SPEED/RADIUS)

            print(str(turns) + " turns left (FIRST)")

            #loop until the robot is again facing its original direction
            while(turns != 0):
                while(obst):

                    #If there is another object in the way in front we must turn again
                    if(sensorDistance.flat_distance(clientID, front_prox_sensor) < 1):
                        print("Hit wall")

                        #drop down the scoop
                        _=move_plow_handle(scoop_left, scoop_right, "DOWN")

                        #turn the robot right again
                        long_lat_turn(True, left_joint, right_joint, remote_bot, heading)
                        t.sleep(0.5)  

                        #increment turns, we now must turn the other way twice to get back to
                        #original heading
                        turns += 1
                        print(str(turns) + " turns left (ADD TURN)")

                        #Move the plow up and drive forward
                        _=move_plow_handle(scoop_left, scoop_right, "UP")
                        _=setWheelVelocity(left_joint, MED_SPEED/RADIUS)
                        t.sleep(0.001)
                        _=setWheelVelocity(right_joint, MED_SPEED/RADIUS)

                    #check again if there is an obstacle to left or right of bot    
                    _, obst = sim.simxReadProximitySensor(clientID, left_prox_sensor, sim.simx_opmode_streaming)[0:2]

                #this is to drive forward 0.5m after the object is no longer seen
                t.sleep(0.5)

                _=setWheelVelocity(left_joint, 0)
                _=setWheelVelocity(right_joint, 0)
                t.sleep(0.5)

                #drop plow and turn 1 turn the opposite direction
                _=move_plow_handle(scoop_left, scoop_right, "DOWN")
                long_lat_turn(False, left_joint, right_joint, remote_bot, heading)
                turns -=1
                _=move_plow_handle(scoop_left, scoop_right, "UP")

                print(str(turns) + " turns left (SUB TURN)")
                if(turns == 0):
                    break

                #drive forward for 1m
                t.sleep(1)
                _=setWheelVelocity(left_joint, MED_SPEED/RADIUS)
                _=setWheelVelocity(right_joint, MED_SPEED/RADIUS)
                t.sleep(1.5)

                _=setWheelVelocity(left_joint, 0)
                _=setWheelVelocity(right_joint, 0)  
                t.sleep(1)

                #if the sensor no longer detects walls then it has fully cleared the obstacle and is now facing the
                #right way. If not the we must turn again and go back up in the loop
                _, final_check = sim.simxReadProximitySensor(clientID, left_prox_sensor, sim.simx_opmode_streaming)[0:2]
                if(final_check == True):
                    _=move_plow_handle(scoop_left, scoop_right, "DOWN")
                    long_lat_turn(False, left_joint, right_joint, remote_bot, heading)
                    _=move_plow_handle(scoop_left, scoop_right, "UP")
                    turns -=1

        elif heading.get_curr_heading_enum() == Orientation.north and heading.get_prev_heading_enum() == Orientation.east:
            print("Hug wall turn left")

            long_lat_turn(False, left_joint, right_joint, remote_bot, heading)
            _=setWheelVelocity(left_joint, 0)
            _=setWheelVelocity(right_joint, 0)
            t.sleep(0.5)

            turns += 1

            obst = 1

            _=move_plow_handle(scoop_left, scoop_right, "UP")

            _=setWheelVelocity(left_joint, MED_SPEED/RADIUS)
            _=setWheelVelocity(right_joint, MED_SPEED/RADIUS)

            print(str(turns) + " turns left (FIRST)")

            while(turns != 0):
                while(obst):

                    if(sensorDistance.flat_distance(clientID, front_prox_sensor) < 1):
                        print("Another Wall Detected")
                        _=move_plow_handle(scoop_left, scoop_right, "DOWN")
                        long_lat_turn(False, left_joint, right_joint, remote_bot, heading)
                        t.sleep(0.5)  

                        turns += 1
                        print(str(turns) + " turns left (ADD TURN)")
                        _=move_plow_handle(scoop_left, scoop_right, "UP")

                        _=setWheelVelocity(right_joint, MED_SPEED/RADIUS)
                        t.sleep(0.001)
                        _=setWheelVelocity(left_joint, MED_SPEED/RADIUS)

                    _, obst = sim.simxReadProximitySensor(clientID, right_prox_sensor, sim.simx_opmode_streaming)[0:2]

                t.sleep(0.5)

                _=setWheelVelocity(left_joint, 0)
                _=setWheelVelocity(right_joint, 0)
                t.sleep(0.5)

                _=move_plow_handle(scoop_left, scoop_right, "DOWN")
                long_lat_turn(True, left_joint, right_joint, remote_bot, heading)
                turns -=1
                _=move_plow_handle(scoop_left, scoop_right, "UP")

                print(str(turns) + " turns left (SUB TURN)")
                if(turns == 0):
                    break

                t.sleep(1)

                _=setWheelVelocity(left_joint, MED_SPEED/RADIUS)
                _=setWheelVelocity(right_joint, MED_SPEED/RADIUS)
                t.sleep(1.5)

                _=setWheelVelocity(left_joint, 0)
                _=setWheelVelocity(right_joint, 0)  
                t.sleep(1)

                _, final_check = sim.simxReadProximitySensor(clientID, right_prox_sensor, sim.simx_opmode_streaming)[0:2]
                if(final_check == True):
                    _=move_plow_handle(scoop_left, scoop_right, "DOWN")
                    long_lat_turn(True, left_joint, right_joint, remote_bot, heading)
                    _=move_plow_handle(scoop_left, scoop_right, "UP")
                    turns -=1

'''
object_detected
-----------------------------------------------------------------
If an object is detected drop scoop, back up, and execute a U-turn (WIP)

param:
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    remote_bot:  Robot ID
    scoop_left:  The left scoop joint ID
    scoop_right: The right scoop joint ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)
    
return: 
'''
def object_detected(left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading):
    #Increment number of obstacles run into in a row
    heading.inc_obj_count()
    print("Objects in a row: " + str(heading.get_obj_count()))

    #If you run into two obstacles in a row after a u-turn you must execute line behaviour
    if(heading.check_obj_count()):
        print("Object avoidance behaviour: LINE")
        line_detected(left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading)
        heading.reset_obj_count()

    #Obstacle avoidance as normal
    else:
        print("Object avoidance behaviour: NORMAL")
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, 0)
        _ = move_plow_handle(scoop_left, scoop_right, "DOWN") 
        t.sleep(0.5)
        _ = setWheelVelocity(left_joint, -MED_SPEED/RADIUS)
        _ = setWheelVelocity(right_joint, -MED_SPEED/RADIUS)
        t.sleep(0.5) 
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, 0)
        t.sleep(0.5)
     
        heading.update_uturn()
        u_turn(left_joint, right_joint, remote_bot, heading.get_curr_heading())
        _ = move_plow_handle(scoop_left, scoop_right, "UP")

        t.sleep(0.5)        

'''
isAtTop
-----------------------------------------------------------------
Check if the robot has reached the top of the map

param:
    remote_bot:  Robot ID
    start:       Start square ID
    
return: True is at top of map, false if not
'''
def isAtTop(remote_bot, start):
    _, vals = sim.simxGetObjectPosition(clientID, remote_bot, start, sim.simx_opmode_streaming)
    if(vals[1] > 10.5):
        return True
    else: return False

'''
isAtBot
-----------------------------------------------------------------
Check if the robot has reached the bottom of the map

param:
    remote_bot:  Robot ID
    start:       Start square ID
    
return: True is at bottom of map, false if not
'''  
def isAtBot(remote_bot, start):
    _, vals = sim.simxGetObjectPosition(clientID, remote_bot, start, sim.simx_opmode_streaming)
    if(vals[1] < 1.5):
        return True
    else: return False

'''
check_reorient
-----------------------------------------------------------------
If the bot is getting closer to an obstacle as it moved, slightly change its direction

param:
    left_joint:  The left wheel joint ID
    right_joint: The right wheel joint ID
    heading:     Desired heading for the bot to turn to 
                 (refer to orientation class for cardinal direction)
    right_prox_sensor: right proximity sensor ID
    left_prox_sensor: left proximity sensor ID

return: 
'''  
def check_reorient(clientID, left_prox_sensor, right_prox_sensor, left_joint, right_joint):
    curr_l = sensorDistance.flat_distance(clientID, left_prox_sensor)
    curr_r = sensorDistance.flat_distance(clientID, right_prox_sensor)

    #if there is an actual value and its closer than 0.7m away
    #on left side then slightly turn to the right
    if(curr_l != 1000.0 and curr_l < 0.7):
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, 0)
        t.sleep(0.25)
        _ = setWheelVelocity(left_joint, MED_SPEED/RADIUS)
        _ = setWheelVelocity(right_joint, 0)
        t.sleep(0.03)
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, 0)

    #if there is an actual value and its closer than 0.7m away
    #on right side then slightly turn to the left
    if(curr_r != 1000.0 and curr_r < 0.7):
        print(curr_r)
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, 0)
        t.sleep(0.25)
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, MED_SPEED/RADIUS)
        t.sleep(0.03)
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, 0)


#main function
if __name__ == "__main__":

    #get client ID
    clientID = startSimulation()
    
    #For map flipping
    vals = []
    orientation = []

    #watchdog for handling timeouts
    watchdog = 0

    if clientID != -1:
        print('Connected to remote API server')
        sim.simxAddStatusbarMessage(clientID, 'Python Script Connected.', sim.simx_opmode_oneshot)

        # Get  ObjectHandles
        _, left_joint      = getObjectHandle('left_joint')
        _, right_joint     = getObjectHandle('right_joint')

        _, front_prox_sensor = getObjectHandle('Front_Prox_Sensor')
        _, left_prox_sensor = getObjectHandle('Left_Prox_Sensor')
        _, right_prox_sensor = getObjectHandle('Right_Prox_Sensor')

        _, line_sensor     = getObjectHandle('Line_Sensor')

        _, plow_lift_left  = getObjectHandle('LeftPlowJoint')
        _, plow_lift_right = getObjectHandle('RightPlowJoint')
        _, scoop_left      = getObjectHandle('ScoopLeftJoint')
        _, scoop_right     = getObjectHandle('ScoopRightJoint') 

        _, remote_bot      =  getObjectHandle('Remote_Bot')      
        _, start       = getObjectHandle('Starting_Line')  

        #Move forward 1m
        _ = setWheelVelocity(left_joint, MAX_SPEED/RADIUS)
        _ = setWheelVelocity(right_joint, MAX_SPEED/RADIUS)
        t.sleep(0.5)

        #stop
        _ = setWheelVelocity(left_joint, 0)
        _ = setWheelVelocity(right_joint, 0)
        t.sleep(1)

        #Open up the plow
        init_plow(plow_lift_left, plow_lift_right)
        t.sleep(1)

        #initialize heading class
        heading = Heading()

        #initizalize sensors to streaming mode
        sim.simxReadProximitySensor(clientID, front_prox_sensor, sim.simx_opmode_streaming)[0:2]
        sim.simxReadProximitySensor(clientID, left_prox_sensor, sim.simx_opmode_streaming)[0:2]
        sim.simxReadProximitySensor(clientID, right_prox_sensor, sim.simx_opmode_streaming)[0:2]

        #turn right
        heading.update_right()
        turn(True, left_joint, right_joint, remote_bot, heading.get_curr_heading())
        t.sleep(1)

        #first loop to mitigate sensor issues
        init = True

        #program ticker
        prog_ticker = 0

        #get current time to see if its been 5 minutes (5 * 60)
        curr_time = t.time()
        while (time.time() - curr_time < 300):

            # ------------- #
            # Read Sensors  #
            # ------------- #
            _ = sim.simxGetObjectGroupData(clientID, 5, 13, sim.simx_opmode_blocking)
            _, detectionState1 = sim.simxReadProximitySensor(clientID, front_prox_sensor, sim.simx_opmode_streaming)[0:2]
            _, detectionState2 = sim.simxReadVisionSensor(clientID, line_sensor, sim.simx_opmode_streaming)[0:2]

            # -------------------- #
            # Top or Bottom of Map #
            # -------------------- #
            if isAtTop(remote_bot, start) and not top_dir_chnage:
                print("Top Line Reached")
                print("Flipping EAST and WEST")

                top_dir_chnage = True
                directionAngles = [0, -1.5128347873687744, 0, 1.5128347873687744]
                
            elif isAtBot(remote_bot, start) and  top_dir_chnage:
                print("Bottom Line Reached")
                print("Flipping EAST and WEST")

                top_dir_chnage = False
                directionAngles = [0, 1.5128347873687744, 0, -1.5128347873687744]

            # ---------- #
            # Re-orient? #
            # ---------- #
            if(prog_ticker % 10 == 0):
                check_reorient(clientID, left_prox_sensor, right_prox_sensor, left_joint, right_joint)

            # -------------- #
            # Check Watchdog #
            # -------------- #
            if watchdog >= 500:
                print("Watchdog Timer Expired")
                _ = setWheelVelocity(left_joint, 0)
                _ = setWheelVelocity(right_joint, 0)
                t.sleep(1)
                watchdog = 0

            # ------------- #
            # Line Detected #
            # ------------- #
            if not detectionState2 and not init:
                print("Line Detected!")
                heading.reset_obj_count()
                line_detected(left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading)

                watchdog = 0

            # --------------- #
            # Object Detected #
            # --------------- #
            elif detectionState1:
                print("Object Detected")
                object_detected(left_joint, right_joint, remote_bot, scoop_left, scoop_right, heading)

                watchdog = 0
            # ------------ #
            # Move Forward #
            # ------------ #
            else:
                _ = setWheelVelocity(left_joint, MED_SPEED/RADIUS)
                _ = setWheelVelocity(right_joint, MED_SPEED/RADIUS)

            watchdog += 1
            init = False
            prog_ticker += 1

        print("5 Minutes has elapsed.")
        sim.simxGetPingTime(clientID)
        sim.simxFinish(clientID)
    else:
        print('Failed connecting to remote API server')
    print('Program ended')
