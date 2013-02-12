# pybo.py
# robert monsen, Feb, 2013
#
# written for python 2.7, using vpython (visual). If you are using
# ubuntu, you can load vpython using the package manager.
# 

from __future__ import division
from visual import *
import pid, glob, os, sys

GRID_SIZE            = 50       # size of the universe
ROBOT_RADIUS         = 1        # radius of the robot
SHOW_PATH            = True     # show the path using a red line
SPEED                = 5        # display speed
CAMERA_FOLLOWS_ROBOT = True     # manipulate the camera to follow the
                                # robot as the simulation progresses
OPACITY              = .5       # how opaque undiscovered objects are

sys.path.append('./maps')
from map import *

sys.path.append('./behaviors')
behaviors = map(__import__, [os.path.basename(x).split('.')[0] for x in glob.glob('behaviors/*.py')])
behaviors = sorted(behaviors, key=lambda x: x.order)

# set obstacles to the obstacle set you prefer from the above choices
obstacles = impossible + random_obst

# initial position, changes as the robot moves
robot_position    = vector(-10, -10, 0)

# goal location (x, y, z)
goal_position     = vector(10, 10, 0)

# utility routines
def distance(a,b):
    return mag(vector(a)-vector(b))

# remove obstacles too near the robot and the goal
obstacles = [(o,r) for o,r in obstacles if (distance(o, goal_position) > ROBOT_RADIUS + r + 1 and 
                                            distance(o, robot_position) > ROBOT_RADIUS + r + 1)]

def normalize_angle(a):
    return math.atan2(math.sin(a),math.cos(a))

def clear_all_objects():
    for obj in scene.objects:
        obj.visible = False
        del obj

def create_grid(base_side=20):
    ygrid = [cylinder(pos=(x, -base_side/2, .05), axis=(0,base_side,0), radius=0.04, color=color.white) for x in range(-base_side//2, base_side//2+1, 1)]
    xgrid = [cylinder(pos=(-base_side/2, y, .05), axis=(base_side,0,0), radius=0.04, color=color.white) for y in range(-base_side//2, base_side//2+1, 1)]

def clear_path(a, there, obstacles):
    # return True if there is a straight path between here and there that will fit a 
    # robot of radius ROBOT_RADIUS
    a = vector(a)
    there = vector(there)
    n = norm(there - a)

    for o,radius in obstacles:
        p = vector(o)
        # compute distance from point to line in vector notation
        pp = a - p
        d = mag(pp - dot(pp,n)*n)
        if d < radius + ROBOT_RADIUS + 0.1:
            return False
    return True

# clean up from the last run
clear_all_objects()
scene.up = (0,0,1)              # z is up, man

# build a grid to drive on
create_grid(40)

# the robot is a little blue dot, with a red arrow indicating the direction it is travelling
robot = cylinder(pos=robot_position, axis=(0,0,1), radius=ROBOT_RADIUS, color=color.blue)
fwd_arrow = arrow(pos=robot_position + vector(0,0,.5), axis=(2,0,0), shaftwidth=0.3, color=color.red)

# create the visual representations of the obstacles
vobsts = [cylinder(pos=opos, axis=(0,0,.1), radius=radius, color=color.red, opacity=OPACITY) for opos,radius in obstacles]

# this dot is to show the currently active goal or obstacle.
target = cylinder(pos=goal_position, axis=(0,0,.2), radius=.3, color=color.white)

# show the goal as a green dot
goal = cylinder(pos=goal_position, axis=(0,0,.1), radius=1, color=color.green)

# set some computational variables. x is the current position, phi is the current direction,
# so the pose is the combination of x and phi
x = robot_position
phi = -math.pi                  # face backwards initially

# this is the base velocity of the robot
base_velocity = 1.0

# this is the actual velocity, which may be a percentage of base_velocity
v = base_velocity

# shorthand for goal
g = goal_position

# time step for simulation
dt = 0.01

# create some pid controllers, one for the robot, and one for the camera
dir_pid = pid.pid(2.0, 0, 0)

if SHOW_PATH:
    pts = points(pos=[], size=.5, color=color.red, retain=50)
    pts.append(x)

seen_obstacles = set()

environment = { 
    'BASE_VELOCITY' : base_velocity, 
    'ROBOT_RADIUS' : ROBOT_RADIUS, 
    'CLEAR_PATH' : clear_path,
    'PHI' : 0
}

while True:
    rate(SPEED/dt)

    # get the distance and angle to goal for computations below
    dist_to_goal = distance(x, g)
    p_goal = math.atan2(g[1] - x[1], g[0] - x[0]) # goal

# this section is the 'sensor' section, which senses any obstacle that is close,
# and adds it to the 'seen_obstacles' set.

    # find the closest obstacle
    for obst in obstacles:
        d_obst = distance(obst[0], x)
        if d_obst < ROBOT_RADIUS + obst[1] + 2:
            seen_obstacles.add(obst)
            vobst = next(x for x in vobsts if x.pos == obst[0])
            if vobst:
                vobst.opacity = 1.0

# This section is the 'behavior' section. It picks a direction (p) for the robot to drive
# look in the 'behaviors' directory to find the behaviors that are being tried here.

    environment['PHI'] = phi
    for behavior in behaviors:
        if behavior.detect(x, g, seen_obstacles, environment):
            p, v = behavior.action(x, g, seen_obstacles, environment)
            break

# this is the PID controller part. Use the error to pick a direction to drive

    # next integration step in the simulation
    x += vector(dt * v * math.cos(phi), dt * v * math.sin(phi), 0)

    # this is the error for the pid controller
    e = normalize_angle(p - phi)

    # pick a new direction to go for next step
    phi = normalize_angle(phi + dt * dir_pid(e))

# Update the position of the robot in the display

    robot.pos = x
    fwd_arrow.pos = robot.pos + vector(0,0,1)
    fwd_arrow.axis = (2*math.cos(phi), 2*math.sin(phi), 0)

    if SHOW_PATH:
        pts.append(x)

# termination of the simulation

    if dist_to_goal < 0.5:
        break

# Change the camera if so desired
    if CAMERA_FOLLOWS_ROBOT:
        scene.center = x
        scene.forward = (math.cos(phi), math.sin(phi), -1)
        scene.range = 20
