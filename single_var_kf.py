#!/usr/bin/python

import pylab
from pylab import array, identity
import random

# the following is based on the code / article at http://greg.czerniak.info/guides/kalman1/
#
# use a kalman filter to determine system's true constant voltage

# inputs
un = array([0]) # control vector
zn = [] # measurement vector

# outputs
xn = array([3]) # newest estimate of true state
pn = array([1]) # newest estimate of average error

# constants
A = array ([1]) # state transition matrix, used to convert last time's state to newest state
B = array([0]) # control matrix
H = array([1]) # observation matrix, used to convert measurement to state
Q = array([0.00001]) # estimated process error covariance
R = array([0.1]) # estimated measurement error covariance
I = identity(1)

# A is 1 as the state is constant
# H is 1 as the measurement is already in voltage 

measure_count = 500
measurements = []
estimations = []
for i in range(measure_count):
    zn = 0.75 + random.random ()

    xn = A*xn + B*un
    pn = A * pn * A.transpose () + Q
    y = zn - H * xn
    S = H * pn * H.transpose () + R
    K = pn * H.transpose () * (1/S)
    xn = xn + K * y
    pn = (I - K * H) * pn

    measurements.append (zn)
    estimations.append (xn)

pylab.figure ()
pylab.plot (measurements, color = 'b', label = 'measured')
pylab.plot ([1.25 for i in range(measure_count)], color = 'r', label = 'true voltage')
pylab.plot (estimations, color = 'g', label = 'kalman')
pylab.legend ()
pylab.show ()
