from pickle import FALSE
import time
import datetime
import math
import random
import itertools
import time
import numpy as np

import pybullet
import pandas as pd
import xlsxwriter

# Number of tests after which to stop
# or enter very large number and end simulation via pressing 'u'
# Will test every combination of test values TESTS times
TESTS = 20

# Force with which to throw/flip coins
FORCE = 0.02 # 0.1 #0.05 #0.02
HVELOCITY = 0

#altura de lançamento em m
ALTURA = 0.03

# How long force is applied in seconds
FORCE_APPLICATION_TIME = 0.05

# Restitution i.e. "bouncyness", keep below 1
# higher = "bouncier"
# ratio of final to initial relative velocity between two objects after collision
RESTITUTION = 0.5
WALL_RESTITUTION = 1

# Friction
LATERAL_FRICTION = 0.8  #0.8
SPINNING_FRICTION = 0.8
ROLLING_FRICTION = 0.0

# Distance between coins in cm, coins will touch with lower values,
# might lead to skewed results
# keep above diameter
DISTANCE_BETWEEN_COINS = 3

# Linear and angular damping
LINEAR_DAMPING = 0.0
ANGULAR_DAMPING = 0.0


# -- Variables below are valid for all tests -- #
# Number of objects per test
OBJECTS = 250

CAIXA = True
if CAIXA:
    OBJECTS = 16

# Time steps for physics simulation in seconds
# Smaller value = more accurate simulation, but more computationally expensive
STEPSIZE = 1/20.0  #120

# Substeps per Timestep
# Higher value = more accurate simulation, but more computationally expensive
SUBSTEPS = 50

# Used to scale up centimeters to decimeters, while keeping accurate physics
# Bullet physics doesn't work well with objects at cm scale
SCALE = 10

# Time after which to stop one run of simulation
# must be high enough for all coins to settle, 
# but low enough to not run (much) longer than needed
CUTOFF = 2

# Slow down simulation each step to see what is happening
# mainly for code refinement/debugging purposes
SLOWDOWN = 0

SHOW_DEBUG_GUI = False

# Diameter of coin IN M


data = {}

# Random distribution of SO(3)
# see http://planning.cs.uiuc.edu/node198.html

sysRand = random.SystemRandom()

pybullet.connect(pybullet.GUI)

def generateRandomQuaternion():
    u1 = sysRand.random()
    u2 = sysRand.random()
    u3 = sysRand.random()
    w = math.sqrt(1 - u1) * math.sin(2 * math.pi * u2)
    x = math.sqrt(1 - u1) * math.cos(2 * math.pi * u2)
    y = math.sqrt(u1) * math.sin(2 * math.pi * u3)
    z = math.sqrt(u1) * math.cos(2 * math.pi * u3)
    return (x, y, z, w)


def simulate(ratio):
    print("")
    print("Razão: " + str(ratio))
    print("")

    RATIO = ratio
    RADIUS = 0.0075

    HEIGHT = RATIO * RADIUS
    DIAMETER = RADIUS*2

    # Mass in gram
    MASS = 0.5487*RATIO + 0.2787

    side = 0
    heads = 0
    tails = 0
    total = 0

    # Set up simulation
    pybullet.setRealTimeSimulation(0)
    pybullet.setTimeStep(STEPSIZE)
    pybullet.resetDebugVisualizerCamera(10, 0, -50, [5, 2, 0])
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, SHOW_DEBUG_GUI)
    pybullet.setPhysicsEngineParameter(numSubSteps=SUBSTEPS)

    for test in range(TESTS):
        print("Test #" + str(test + 1))
        stopSim = False

        # Turn off rendering for building world
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

        # Remove all objects
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.80665 * SCALE)

        # Create ground plane
        pybullet.createCollisionShape(pybullet.GEOM_PLANE)
        pybullet.createMultiBody(0, 0)
        pybullet.changeDynamics(0, -1, restitution=0.1, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)
        
        #Create walls

        if CAIXA:
            wall_collision_shapeY = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=(0.003 * SCALE, 0.31/2 * SCALE, 0.31/2 *SCALE))
            wall_collision_shapeX = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=(0.31/2* SCALE, 0.003 * SCALE, 0.31/2*SCALE))
            wall1 = pybullet.createMultiBody(0., wall_collision_shapeY, -1, basePosition=(-0.31/2 * SCALE, 0, 0.))
            wall2 = pybullet.createMultiBody(0., wall_collision_shapeY, -1, basePosition=(0.31/2 * SCALE, 0, 0.))
            wall3 = pybullet.createMultiBody(0., wall_collision_shapeX, -1, basePosition=(0 * SCALE, -0.31/2 * SCALE, 0.))
            wall4 = pybullet.createMultiBody(0., wall_collision_shapeX, -1, basePosition=(0 * SCALE, 0.31/2 * SCALE, 0.))
            pybullet.changeDynamics(wall1, -1, restitution=WALL_RESTITUTION, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)
            pybullet.changeDynamics(wall2, -1, restitution=WALL_RESTITUTION, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)
            pybullet.changeDynamics(wall3, -1, restitution=WALL_RESTITUTION, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)
            pybullet.changeDynamics(wall4, -1, restitution=WALL_RESTITUTION, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)


        # Create collision shape for coin, which will be used for all bodies
        colCylinder = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=RADIUS * SCALE, height= HEIGHT * SCALE)

        # Create bodies with random forces applied
        for i in range(OBJECTS):

            if not CAIXA:
                x = pybullet.createMultiBody(baseMass= MASS / 1000 * SCALE, baseCollisionShapeIndex=colCylinder,
                                        basePosition=[i % 10 / 100 * DISTANCE_BETWEEN_COINS * SCALE, i / 10 / 100 * DISTANCE_BETWEEN_COINS * SCALE, ALTURA * SCALE], 
                                        baseOrientation=generateRandomQuaternion())
            else:
                p  = [[-9, -9], [-3, -9], [3, -9], [9, -9], [-9, -3], [-3, -3], [3, -3], [9, -3], [-9, 3], [-3, 3], [3, 3], [9, 3], [-9, 9], [-3, 9], [3, 9], [9, 9]]
                x = pybullet.createMultiBody(baseMass= MASS / 1000 * SCALE, baseCollisionShapeIndex=colCylinder,
                                        basePosition=[p[i][0]*SCALE/100,  p[i][1] * SCALE/100, ALTURA * SCALE], 
                                        baseOrientation=generateRandomQuaternion())
            

            pybullet.changeDynamics(x, -1, linearDamping=LINEAR_DAMPING, angularDamping=ANGULAR_DAMPING, restitution=RESTITUTION,
                                    lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)

            #pybullet.changeDynamics(x, -1, linearDamping=0, angularDamping=0, restitution=1,
                                    #lateralFriction=1, spinningFriction=1, rollingFriction=1)

            pybullet.applyExternalForce(x, -1, [(sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCE_APPLICATION_TIME,
                                                (sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCE_APPLICATION_TIME,
                                                (sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCE_APPLICATION_TIME],
                                    [(sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE,
                                    (sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE,
                                    (sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE],
                                    pybullet.LINK_FRAME)

            pybullet.resetBaseVelocity(x, (HVELOCITY * SCALE, 0, 0), (0, 0, 0))


        # Turn on rendering again
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

        for i in range(int(1/STEPSIZE * CUTOFF)):
            pybullet.stepSimulation()
            time.sleep(SLOWDOWN)
            keys = pybullet.getKeyboardEvents()
            for k in keys:
                if(k == 117):
                    if not stopSim:
                        stopSim = True
                        print("Will exit after this test finished")

        # First object is ground plane -> ignore
        for i in range(1, OBJECTS + 1):
            pos, angle = pybullet.getBasePositionAndOrientation(i)
            angle = pybullet.getEulerFromQuaternion(angle)
            total += 1
            # Check how coin landed:
            angle = (angle[0] + 2 * math.pi) % (2 * math.pi)

            # Side
            if math.pi / 2 - (math.pi / 2 - 1.3) <= angle <= math.pi / 2 + (math.pi / 2 - 1.3) or math.pi / 2 * 3 - (math.pi / 2 - 1.3) <= angle <= math.pi / 2 * 3 + (math.pi / 2 - 1.3):
                side += 1

            # Heads
            if angle <= 1.3 or 2 * math.pi - 1.3 <= angle:
                heads += 1

            # Tails
            if math.pi - 1.3 <= angle <= math.pi + 1.3:
                tails += 1

        if(stopSim):
            break

    Ps = side/total
    data[RATIO] = Ps
    return Ps


razoes = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5]

for i in razoes:
    simulate(i)

print("")
print(data)

workbook = xlsxwriter.Workbook('simulacoes/teste2.xlsx')
worksheet = workbook.add_worksheet()

for i in range(len(razoes)):
    worksheet.write(i, 0, razoes[i])
    worksheet.write(i, 1, data[razoes[i]])

workbook.close()
