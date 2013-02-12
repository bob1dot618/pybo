from __future__ import division
from visual import *

order=2

def distance(a,b):
    return mag(vector(a)-vector(b))

def detect(x, g, obstacles, env):
    cp = env['CLEAR_PATH'](x,g,obstacles)
    return not cp

def action(x, g, obstacles, env):
    d,(o,r) = min((distance(x,obst[0]), obst) for obst in obstacles)
    if d > r + env['ROBOT_RADIUS'] + 2:
        # just go-to-goal
        return math.atan2(g[1] - x[1], g[0] - x[0]), env['BASE_VELOCITY']
    elif d > r + env['ROBOT_RADIUS'] + 1:
        z = rotate(vector(o) - x, math.pi/2.0, (0,0,1))
        return math.atan2(z[1], z[0]), env['BASE_VELOCITY']/2
    elif d > r + env['ROBOT_RADIUS'] + 0.5:
        z = rotate(vector(o) - x, 2*math.pi/3, (0,0,1))
        return math.atan2(z[1], z[0]), env['BASE_VELOCITY']/3
    elif d > r + env['ROBOT_RADIUS']:
        z = x - vector(o)
        return math.atan2(z[1], z[0]), env['BASE_VELOCITY']/4
    else:
        return 0, 0
