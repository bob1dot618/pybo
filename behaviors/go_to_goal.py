from __future__ import division
from visual import *

order=1

def detect(x, g, obstacles, env):
    return env['CLEAR_PATH'](x, g, obstacles)

def action(x, g, obstacles, env):
    return math.atan2(g[1] - x[1], g[0] - x[0]), env['BASE_VELOCITY']
