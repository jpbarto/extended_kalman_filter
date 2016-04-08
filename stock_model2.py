#!/usr/bin/python

# based on the article found at http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx

from datetime import date
from time import time
from math import sqrt
import pylab
from pylab import array, random
from yahoo_finance import Share

start_date = date.fromtimestamp (time () - 60*60*24*30).strftime ('%Y-%m-%d')
end_date = date.today ().strftime ('%Y-%m-%d')
share = Share ('TWTR')
share_history = share.get_historical (start_date, end_date)
readings = []
for day in share_history:
    readings.append (float(day['Close']))

# kalman filter implementation
# ============================
process_variance = 1e-5
estimated_measurement_variance = 0.1**2
posteri_estimate = 0.0
posteri_error_estimate = 1.0
estimates = []
estimate_errors = []

for reading in readings:
    # predict
    priori_estimate = posteri_estimate
    priori_error_estimate = posteri_error_estimate + process_variance

    # update
    kalman_gain = priori_error_estimate / (priori_error_estimate + estimated_measurement_variance)
    posteri_estimate = priori_estimate + kalman_gain + (reading - priori_estimate)
    posteri_error_estimate = (1 - kalman_gain) * priori_error_estimate

    estimates.append (posteri_estimate)
    estimate_errors.append (posteri_error_estimate)

pylab.figure ()
pylab.plot(readings, color = 'b', label = 'actual location')
pylab.plot(estimates, color = 'r', label = 'estimate location')
pylab.legend ()
pylab.show ()
