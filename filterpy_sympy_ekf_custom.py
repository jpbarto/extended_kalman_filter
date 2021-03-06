#!/usr/bin/python

import pylab
from pylab import random, eye, array, asarray
from math import sqrt
from filterpy.kalman import ExtendedKalmanFilter
import sympy

# the following is a use of the FilterPY library based on the developers documentation found at
# http://nbviewer.ipython.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/09_Extended_Kalman_Filters/Extended_Kalman_Filters.ipynb

class RadarNet (object):
    sensors = [[0,0], [5,0], [5,5]]
    target = [3,2]

    def get_reading (self):
        reading = []
        t = self.target
        for s in self.sensors:
            r = sqrt ((t[0] - s[0])**2 + (t[1] - s[1])**2) + (random () - 0.5)
            reading.append ([s[0], s[1], r])
        return reading

    def step (self):
        self.target[0] += (5 * (random () - 0.5))
        self.target[1] += (5 * (random () - 0.5))


est_x, est_y = sympy.symbols ('est_x est_y')
s1_x, s1_y, s2_x, s2_y, s3_x, s3_y = sympy.symbols('s1_x s1_y s2_x s2_y s3_x s3_y')

H = sympy.Matrix ([[sympy.sqrt((est_x - s1_x)**2 + (est_y - s1_y)**2)], 
                   [sympy.sqrt((est_x - s2_x)**2 + (est_y - s2_y)**2)], 
                   [sympy.sqrt((est_x - s3_x)**2 + (est_y - s3_y)**2)]])
state = sympy.Matrix ([est_x, est_y])
jacobian = H.jacobian (state)
def HJacobian_at (x):
    values = {est_x: x.item(0), est_y: x.item(1)}
    values[s1_x] = radar.sensors[0][0]
    values[s1_y] = radar.sensors[0][1]
    values[s2_x] = radar.sensors[1][0]
    values[s2_y] = radar.sensors[1][1]
    values[s3_x] = radar.sensors[2][0]
    values[s3_y] = radar.sensors[2][1]
    return asarray(jacobian.evalf (subs = values))

def hx (x):
    ranges = []
    est_x = x.item (0)
    est_y = x.item (1)

    for s in radar.sensors:
        ranges.append ([sqrt((est_x - s[0])**2 + (est_y - s[1])**2)])

    return ranges

radar = RadarNet ()
ekf = ExtendedKalmanFilter (dim_x = 2, dim_z = 3)

ekf.x = array([[1.0], [1.0]])
# ekf.x = array([[radar.pos, radar.vel+100, radar.alt+1000]]).T
# ekf.x = array([[0.0, 0.0, 0.0]]).T

ekf.F = eye(2)
# ekf.R = radar.target[1] * 0.05
ekf.R *= 10
ekf.Q *= 0.0001
ekf.P *= 50

truth_x = []
truth_y = []

estimated_x = []
estimated_y = []

for i in range(1000):
    readings = array(radar.get_reading ())

    truth_x.append (radar.target[0])
    truth_y.append (radar.target[1])

    z = []
    for r in readings:
        z.append ([r[2]])
    z = array(z)
    ekf.update (z, HJacobian_at, hx)

    estimated_x.append (ekf.x[0])
    estimated_y.append (ekf.x[1])

    ekf.predict ()

    # radar.step ()

pylab.figure ()
pylab.plot (truth_x, color = 'b', label = 'actual position')
pylab.plot (truth_y, color = 'g', label = 'actual altitude')
pylab.plot (estimated_x, color = 'r', label = 'estimated position')
pylab.plot (estimated_y, color = 'y', label = 'estimated altitude')
pylab.legend ()
pylab.show ()

