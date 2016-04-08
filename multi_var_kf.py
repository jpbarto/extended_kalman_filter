#!/usr/bin/python

import sys
import pylab
from pylab import matrix, identity, zeros, linalg, array
import random
import math

# the following is based on the code / article at http://greg.czerniak.info/guides/kalman1/
#
# use a kalman filter to determine system's true constant voltage

# the following cannon ball class is directly copied from the multi variable article
# Simulates the classic physics problem of a cannon shooting a ball in a
# parabolic arc.  In addition to giving "true" values back, you can also ask
# for noisy values back to test Kalman filters.
class CannonBall:
  #--------------------------------VARIABLES----------------------------------
  angle = 45 # The angle from the ground to point the cannon.
  muzzle_velocity = 100 # Muzzle velocity of the cannon.
  gravity = [0,-9.81] # A vector containing gravitational acceleration.
  # The initial velocity of the cannonball
  velocity = [muzzle_velocity*math.cos(angle*math.pi/180), muzzle_velocity*math.sin(angle*math.pi/180)]
  loc = [0,0] # The initial location of the cannonball.
  acceleration = [0,0] # The initial acceleration of the cannonball.
  #---------------------------------METHODS-----------------------------------
  def __init__(self,_timeslice,_noiselevel):
    self.timeslice = _timeslice
    self.noiselevel = _noiselevel
  def add(self,x,y):
    return x + y
  def mult(self,x,y):
    return x * y
  def GetX(self):
    return self.loc[0]
  def GetY(self):
    return self.loc[1]
  def GetXWithNoise(self):
    return random.gauss(self.GetX(),self.noiselevel)
  def GetYWithNoise(self):
    return random.gauss(self.GetY(),self.noiselevel)
  def GetXVelocity(self):
    return self.velocity[0]
  def GetYVelocity(self):
    return self.velocity[1]
  # Increment through the next timeslice of the simulation.
  def Step(self):
    # We're gonna use this vector to timeslice everything.
    timeslicevec = [self.timeslice,self.timeslice]
    # Break gravitational force into a smaller time slice.
    sliced_gravity = map(self.mult,self.gravity,timeslicevec)
    # The only force on the cannonball is gravity.
    sliced_acceleration = sliced_gravity
    # Apply the acceleration to velocity.
    self.velocity = map(self.add, self.velocity, sliced_acceleration)
    sliced_velocity = map(self.mult, self.velocity, timeslicevec )
    # Apply the velocity to location.
    self.loc = map(self.add, self.loc, sliced_velocity)
    # Cannonballs shouldn't go into the ground.
    if self.loc[1] < 0:
      self.loc[1] = 0


# constants
g = -9.81
dt = 0.1
A = matrix ([[1, dt, 0, 0], [0,1,0,0], [0,0,1,dt], [0,0,0,1]]) # state transition matrix, used to convert last time's state to newest state
B = matrix([[0,0,0,0], [0,0,0,0], [0,0,1,0], [0,0,0,1]]) # control matrix
H = identity (4) # observation matrix, used to convert measurement to state
Q = zeros ((4,4)) # estimated process error covariance
R = identity(4) * 0.2 # estimated measurement error covariance
I = identity (4)

# inputs
un = matrix([[0], [0], [0.5 * g * dt * dt], [g * dt]]) # control vector
zn = [] # measurement vector

# outputs
xn = matrix([[0], [100 * math.cos (math.pi/4)], [500], [100 * math.sin (math.pi / 4)]]) # newest estimate of true state
pn = identity (4) # newest estimate of average error

cannon_ball = CannonBall (dt, 30)
measure_count = 144
measurements = []
truth = []
estimations = []
for i in range(measure_count):
    realX = cannon_ball.GetX ()
    velX = cannon_ball.GetXVelocity ()
    realY = cannon_ball.GetY ()
    velY = cannon_ball.GetYVelocity ()
    noiseX = cannon_ball.GetXWithNoise ()
    noiseY = cannon_ball.GetYWithNoise ()

    truth.append ([realX, realY])
    measurements.append ([noiseX, noiseY])

    cannon_ball.Step ()

    estimations.append ([xn.item (0), xn.item(2)])

    zn = matrix([[noiseX], [cannon_ball.GetXVelocity ()], [noiseY], [cannon_ball.GetYVelocity ()]])

    xn = A * xn + B * un
    pn = (A * pn) * A.transpose () + Q
    y = zn - H * xn
    S = H * pn * H.transpose () + R
    K = pn * H.transpose () * linalg.inv (S)
    xn = xn + K * y
    pn = (I - K * H) * pn

measurements = array (measurements)
truth = array (truth)
estimations = array (estimations)
pylab.figure ()
pylab.plot (measurements[0:measure_count, 0], measurements[0:measure_count, 1], color = 'b', linestyle = '--', label = 'measured')
pylab.plot (truth[0:measure_count, 0], truth[0:measure_count, 1], color = 'r', label = 'true')
pylab.plot (estimations[0:measure_count, 0], estimations[0:measure_count, 1], color = 'g', label = 'kalman')
pylab.legend ()
pylab.show ()
