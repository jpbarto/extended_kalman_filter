#!/usr/bin/python

import pylab
from pylab import randn, eye, array, asarray
from math import sqrt
from filterpy.kalman import ExtendedKalmanFilter

# the following is a use of the FilterPY library based on the developers documentation found at
# http://nbviewer.ipython.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/09_Extended_Kalman_Filters/Extended_Kalman_Filters.ipynb

class Flight (object):
    """ Simulates the radar signal returns from an object flying 
    at a constant altityude and velocity in 1D. 
    """
    def __init__(self, dt, pos, vel, alt):
        self.pos = pos
        self.vel = vel
        self.alt = alt
        self.dt = dt
        
    def get_range(self):
        """ Returns slant range to the object. Call once for each
        new measurement at dt time from last call.
        """
        
        # add some process noise to the system
        vel = self.vel  + 5*randn()
        alt = self.alt + 10*randn()
        self.pos = self.pos + vel*self.dt
    
        # add measurment noise
        err = self.pos * 0.05*randn()
        slant_dist = sqrt(self.pos**2 + alt**2)
        
        return slant_dist + err

def HJacobian_at (x):
    horiz_dist = x[0,0]
    altitude = x[2,0]
    denom = sqrt(horiz_dist**2 + altitude**2)
    return array([[horiz_dist/denom, 0.0, altitude/denom]])

def hx (x):
    return (x[0,0]**2 + x[2,0]**2)**0.5

dt = 0.05

flight = Flight (dt, pos = 0.0, vel = 100.0, alt = 1000.0)
ekf = ExtendedKalmanFilter (dim_x = 3, dim_z = 1)

ekf.x = array([[flight.pos, flight.vel+100, flight.alt+1000]]).T
# ekf.x = array([[flight.pos, flight.vel+100, flight.alt+1000]]).T
# ekf.x = array([[0.0, 0.0, 0.0]]).T

ekf.F = eye(3) + array([[0,1,0], [0,0,0], [0,0,0]])*dt
ekf.R = flight.alt * 0.05
ekf.Q = array ([[0,0,0], [0,1,0], [0,0,1]]) * 0.001
ekf.P *= 50

truth_x = []
truth_y = []

estimated_x = []
estimated_y = []

for i in range(int(20/dt)):
    r = flight.get_range ()

    truth_x.append (flight.pos)
    truth_y.append (flight.alt)

    ekf.update (array([[r]]), HJacobian_at, hx)

    estimated_x.append (ekf.x[0])
    estimated_y.append (ekf.x[2])

    ekf.predict (0)

pylab.figure ()
pylab.plot (truth_x, color = 'b', label = 'actual position')
pylab.plot (truth_y, color = 'g', label = 'actual altitude')
pylab.plot (estimated_x, color = 'r', label = 'estimated position')
pylab.plot (estimated_y, color = 'y', label = 'estimated altitude')
pylab.legend ()
pylab.show ()
