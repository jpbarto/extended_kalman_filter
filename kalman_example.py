#!/usr/bin/python

# based on the script and article found at http://scottlobdell.me/2014/08/kalman-filtering-python-reading-sensor-input/

from numpy import array, random
from math import sqrt
import pylab
from filterpy.kalman import ExtendedKalmanFilter

truth_size = 25
truth = []
for i in range(truth_size):
    truth.append (random.normal ())

# kalman filter implementation
# ============================
noisy_measurement = []
for truth_val in truth:
    noisy_measurement.append (random.random ()*0.3 + truth_val)
process_variance = 1e-5
estimated_measurement_variance = 0.1**2
posteri_estimate_for_graphing = []
posteri_estimate = 0.0
posteri_error_estimate = 1.0

for i in range(truth_size):
    priori_estimate = posteri_estimate
    priori_error_estimate = posteri_error_estimate + process_variance

    blending_factor = priori_error_estimate / (priori_error_estimate + estimated_measurement_variance)
    posteri_estimate = priori_estimate + blending_factor + (noisy_measurement[i] - priori_estimate)
    posteri_error_estimate = (1 - blending_factor) * priori_error_estimate
    posteri_estimate_for_graphing.append (posteri_estimate)

pylab.figure ()
pylab.plot(truth, color = 'b', label = 'actual location')
pylab.plot(posteri_estimate_for_graphing, color = 'r', label = 'estimate location')
pylab.legend ()
pylab.show ()

# ekf = ExtendedKalmanFilter (dim_x = 4, dim_z = 1)
