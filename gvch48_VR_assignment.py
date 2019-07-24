# -*- coding: utf-8 -*-
"""
Created on Wed Mar  6 16:13:34 2019
VR Coursework
@author: gvch48
Python 3.6.4

Place this file in same folder as IMUData.csv
Run the file and a simple UI will appear. follow the instructions in the UI
(you probably just want to answer y to each prompt)
"""

import csv
import math
from math import cos, sin, asin, atan2, acos, pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation


"""
###   PART 1 FUNCTIONS
"""

def getIMUDataFromCSV(filename):
    IMUData = []
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file,delimiter=',')
        for row in csv_reader:
            IMUData.append(row)
    IMUData.pop(0)              #removes the labels from the data
    for row in range(len(IMUData)):
        for col in range(10):
            IMUData[row][col] = float(IMUData[row][col])
    return IMUData

#input is the output of getIMUDataFromCSV
def convertDegreesToRads(IMUData):
    for index in range(len(IMUData)):
        IMUData[index][1] *= pi/180
        IMUData[index][2] *= pi/180
        IMUData[index][3] *= pi/180

# replaced with normaliseDataValues
def normaliseAccelerometerValues(IMUData):
    for index in range(len(IMUData)):
        mag = math.sqrt( math.pow(IMUData[index][4],2) + math.pow(IMUData[index][5],2) + math.pow(IMUData[index][6],2) )
        if mag == 0:
            mag = 1
        IMUData[index][4] /= mag
        IMUData[index][5] /= mag
        IMUData[index][6] /= mag
 
#replaced with normaliseDataValues       
def normaliseMagnetometerValues(IMUData):
    for index in range(len(IMUData)):
        mag = math.sqrt( math.pow(IMUData[index][7],2) + math.pow(IMUData[index][8],2) + math.pow(IMUData[index][9],2) )
        if mag == 0:
            mag = 1
        IMUData[index][7] /= mag
        IMUData[index][8] /= mag
        IMUData[index][9] /= mag
 
# more general version of above function that can be used after  IMUData is separated   
#inputs are lists of data
def normaliseDataValues(dataX, dataY, dataZ):
    normDataX = []
    normDataY = []
    normDataZ = []
    for index in range(len(dataX)):
        mag = math.sqrt(math.pow(dataX[index], 2) + math.pow(dataY[index], 2) + math.pow(dataZ[index], 2) )
        if(mag == 0):
            mag = 1
        normDataX.append(dataX[index]/mag)
        normDataY.append(dataY[index]/mag)
        normDataZ.append(dataZ[index]/mag)
    
    return normDataX, normDataY, normDataZ

#input is the output of getIMUDataFromCSV       
def separateIMUData(IMUData):
    time = []
    gyroX = []
    gyroY = []
    gyroZ = []
    accelX = []
    accelY = []
    accelZ = []
    magnetX = []
    magnetY = []
    magnetZ = []
    for row in range(len(IMUData)):
        time.append(IMUData[row][0])
        gyroX.append(IMUData[row][1])
        gyroY.append(IMUData[row][2])
        gyroZ.append(IMUData[row][3])
        accelX.append(IMUData[row][4])
        accelY.append(IMUData[row][5])
        accelZ.append(IMUData[row][6])
        magnetX.append(IMUData[row][7])
        magnetY.append(IMUData[row][8])
        magnetZ.append(IMUData[row][9])
    return time, gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magnetX, magnetY, magnetZ
        
#adapted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
#inputs reordered to match this standard: http://www.euclideanspace.com/maths/standards/index.htm
#X -> Z
#Y -> X
#Z -> Y
# inputs are individual numbers
def toQuaternion(yaw_, pitch_, roll_):
    
    roll = yaw_
    yaw = pitch_
    pitch = roll_
    
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    
    w = cy * cp * cr - sy * sp * sr
    x = cy * cp * sr + sy * sp * cr
    y = sy * cp * cr + cy * sp * sr
    z = cy * sp * cr - sy * cp * sr
    
    return [w,x,y,z]

#adapts java code from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
#outputs reordered to match this standard: http://www.euclideanspace.com/maths/standards/index.htm
#X -> Z
#Y -> X
#Z -> Y
# quaternion is a list
def toEulerAngle(quaternion):
    w, x, y, z = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    test = x*y + w*z
    sqw = w*w
    sqx = x*x
    sqy = y*y
    sqz = z*z
    
    unit = sqw + sqx + sqy + sqz  #if normalised is one, otherwise is correction factor
    if (test > 0.4985 * unit):       #singularity at north pole
        yaw = 2 * atan2(x, w)
        pitch = pi/2
        roll = 0
        
    elif(test < -0.4985 * unit):     # singularity at south pole
        yaw = 2 * atan2(x, w)
        pitch = -pi/2
        roll = 0
        
    else:
        #yaw  aka heading
        v1 =  2*(y*w - x*z)
        v2 = sqx - sqy - sqz + sqw
        yaw = atan2(v1,v2)
        
        #pitch aka attitude
        pitch = asin(2*test/unit)
        
        #roll  aka bank
        u1 = 2*(x*w - y*x)
        u2 = -sqx + sqy - sqz + sqw
        roll = atan2(u1,u2)
 
    ## euler angles greater than 144 deg or less than -144 deg are errors, the code below fixes them   
    if(yaw > pi*0.8):
        yaw = pi - yaw
    elif(yaw < -pi*0.8):
        yaw += pi
        
    if(pitch > pi*0.8):
        pitch = pi - pitch
    elif(pitch < -pi*0.8):
        pitch += pi
        
    if(roll > pi*0.8):
        roll = pi - roll
    elif(roll  < -pi*0.8):
        roll += pi    

        
    ##outputs 
    yaw_ = roll
    pitch_ = yaw
    roll_ = pitch

    return [yaw_, pitch_, roll_]

#quaternion is a list
def conjugateQuaternion(quaternion):
    w = quaternion[0]
    x = -quaternion[1]
    y = -quaternion[2]
    z = -quaternion[3]
    return [w,x,y,z]

#quatA, quatB are lists
def quaternionProduct(quatA, quatB):
    w = quatB[0] * quatA[0] - quatB[1] * quatA[1] - quatB[2] * quatA[2] - quatB[3] * quatA[3]
    x = quatB[0] * quatA[1] + quatB[1] * quatA[0] - quatB[2] * quatA[3] + quatB[3] * quatA[2]
    y = quatB[0] * quatA[2] + quatB[1] * quatA[3] + quatB[2] * quatA[0] - quatB[3] * quatA[1]
    z = quatB[0] * quatA[3] - quatB[1] * quatA[2] + quatB[2] * quatA[1] + quatB[3] * quatA[0]
    return [w,x,y,z]

# not used
# quatA, quatB are lists
def quaternionSum(quatA, quatB):
    w = quatA[0] + quatB[0]
    x = quatA[1] + quatB[1]
    y = quatA[2] + quatB[2]
    z = quatA[3] + quatB[3]
    return [w,x,y,z]


"""
###   PART 2 FUNCTIONS
"""
#inputs: normalised axis vector, angle in rads
def axisAngleToQuaternion(axis, angle):
    w = cos(angle * 0.5)
    x = axis[0] * sin(angle * 0.5)
    y = axis[1] * sin(angle * 0.5)
    z = axis[2] * sin(angle * 0.5)
    return [w,x,y,z]


#implementation from Madgwick's paper
# not used. see Version 2 below
def orientationQuatsFromGyroOnlyVersion1(time, gyroX, gyroY, gyroZ):
    quats = [None]*len(time)
    initialProduct = quaternionProduct([0.5,0,0,0], [0,gyroY[0],gyroZ[0],gyroX[0]])
    quats[0] = quaternionSum([1,0,0,0], [x/256 for x in initialProduct])
    
    for row in range(1,len(time)):
        prevEst = quats[row-1]
        deltaT = (time[row] - time[row-1])
        halfPrevEst = [x * 0.5 for x in prevEst]
        gyroQuat = [0,gyroY[row],gyroZ[row],gyroX[row]]
        derivative = quaternionProduct(halfPrevEst, gyroQuat)
        derivativeDeltaT  = [x * deltaT for x in derivative]
        quats[row] = quaternionSum(prevEst, derivativeDeltaT)
    
    return quats


# vector is a list
def vectorMag(vector):
    dotProduct = 0
    for x in range(len(vector)):
        dotProduct += math.pow(vector[x], 2)
    return math.sqrt(dotProduct)

# computes the q(v,theta) part of equation (2) from Lavelles Paper
# inputs are each lists of dataValues (columns of Dataset)
def computeVThetaQuats(time, gyroX, gyroY, gyroZ):
    vThetaQuats = [None]*len(time)
    
    ## t = 0
    gyro_observed = [gyroY[0], gyroZ[0], gyroX[0]]
    mag = vectorMag(gyro_observed)
    theta = mag  * 1/256
    if(mag != 0):
        v = [x/mag for x in gyro_observed]
    else:
        v = [x for x in gyro_observed]
    vThetaQuats[0] = axisAngleToQuaternion(v, theta)
    
    for val in range(1,len(time)):
        gyro_observed = [gyroY[val], gyroZ[val], gyroX[val]]
        mag = vectorMag(gyro_observed)
        deltaT = (time[val] - time[val-1])
        theta = mag * deltaT
        if(mag != 0):
            v = [x/mag for x in gyro_observed]
        else:
            v = [x for x in gyro_observed]
        vThetaQuats[val] = axisAngleToQuaternion(v, theta)
        
    return vThetaQuats

#implementation from LaValle's book
#inputs are output of above function, and list of time values
def orientationQuatsFromGyroOnlyVersion2(vThetaQuats, time):
    oriQuats = [0]*len(time)
    
    oriQuats[0] = [1,0,0,0]    #quaternionProduct([1,0,0,0], vThetaQuats[0])
    
    for row in range(1,len(time)):
        oriQuats[row] = quaternionProduct(oriQuats[row-1], vThetaQuats[row-1])
    
    return oriQuats

# input: list of quaternions (4 element lists)
# outputs: lists  of yaws, pitches and rolls at each timestep
def convertOrientationQuatsToEulerAngles(quats):
    yaws = []
    pitches = []
    rolls = []
    for row in range(len(quats)):
        y, p, r = toEulerAngle(quats[row])
        yaws.append(y)
        pitches.append(p)
        rolls.append(r)
    return yaws, pitches, rolls


"""
###   PART 3 FUNCTIONS
"""

#input is a vector (stored as a list)
def computeTiltAxis(a):
    return [ a[2], 0 , -a[0] ]

# inputs are 2 vectors (stored as lists)
def computeAngleBetweenVectors(u,v):
    uMag = vectorMag(u)
    vMag = vectorMag(v)
    dotProduct = 0
    for x in range(len(u)):
        dotProduct += u[x]*v[x]
    return acos(dotProduct/(uMag*vMag))
    
    
#inputs are a list of data, and an integer n
def averageOverNSamples(data, n):
    if n < 2:
        return data
    else:
        averagedData = [0]*len(data)
        for val in range(len(data)):
            if val < n//2 :
                averagedData[val] = data[val]
            elif val > len(data) - n//2:
                averagedData[val] = data[val]
            else:
                for x in range(val-n//2, val+n//2):
                    averagedData[val] += data[x]
                averagedData[val] /= (1+ 2*(n//2))
                
        return averagedData


def orientationQuatsWithTiltCorrection(alpha, gyroOriQuats, vThetaQuats, accelX, accelY, accelZ):
    TiltCorrectedOrientationQuats = [None]*len(gyroOriQuats)
    
    ##initial value
    TiltCorrectedOrientationQuats[0] = gyroOriQuats[0]
    
    for val in range(1,len(gyroOriQuats)):
        mag = vectorMag([accelY[val], accelZ[val], accelX[val] ])  #averaged vectors arent normalised
        a_observed = [0,accelY[val]/mag ,accelZ[val]/mag ,accelX[val]/mag]
        
        q = quaternionProduct(TiltCorrectedOrientationQuats[val-1], vThetaQuats[val])
        a = quaternionProduct(conjugateQuaternion(q), quaternionProduct(a_observed, q))
        tiltAxis = computeTiltAxis([a[1],a[2],a[3]])
        phi = computeAngleBetweenVectors([a[1],a[2],a[3]], [0,1,0])   #remember: Y,Z,X
        quat  = axisAngleToQuaternion(tiltAxis, -alpha*phi)
        TiltCorrectedOrientationQuats[val] = quaternionProduct(quat, gyroOriQuats[val])
        
    return TiltCorrectedOrientationQuats

"""
###   PART 4 FUNCTIONS
"""

def orientationQuatsWithYawTiltCorrection(alpha, TiltCorrectedOrientationQuats, vThetaQuats, magnetX, magnetY, magnetZ):
    YawTiltCorrectedOrientationQuats = [None]*len(TiltCorrectedOrientationQuats)
    
    YawTiltCorrectedOrientationQuats[0] = TiltCorrectedOrientationQuats[0]
    
    m_ref_observed = [0, magnetY[0], magnetZ[0], magnetX[0]]
    m_ref = quaternionProduct(conjugateQuaternion(TiltCorrectedOrientationQuats[0]), quaternionProduct(m_ref_observed, TiltCorrectedOrientationQuats[0]))
    theta_ref = atan2(m_ref[1], m_ref[3])
    
    for val in range(1, len(TiltCorrectedOrientationQuats)):
        mag = vectorMag([magnetY[val], magnetZ[val], magnetX[val] ]) # averaged vectors arent normalised
        m_observed = [0, magnetY[val]/mag, magnetZ[val]/mag, magnetX[val]/mag ]
        q = quaternionProduct(YawTiltCorrectedOrientationQuats[val-1], vThetaQuats[val])
        m = quaternionProduct(conjugateQuaternion(q), quaternionProduct(m_observed, q))
        theta = atan2(m[1], m[3])
        quat = axisAngleToQuaternion([0,1,0], -alpha*(theta-theta_ref))  #remember: Y,Z,X
        
        YawTiltCorrectedOrientationQuats[val] = quaternionProduct(quat, TiltCorrectedOrientationQuats[val])
    
    return YawTiltCorrectedOrientationQuats

"""
###   PART 6 FUNCTIONS
"""

#assumes the averaged accel data approximates gravity
def computeLinearAcceleration(accelData, averagedAccelData):
    a_l = []
    for val in range(len(accelData)):
        a_l.append(9.81*(accelData[val] - averagedAccelData[val]) )
    
    return a_l

def integrateLinearAcceleration(time, accelData):
    w = [None]*len(time)
    w[0] = 0
    for val in range(1, len(time)):
        w[val] = w[val-1] + accelData[val]*(time[val] - time[val-1])
    
    return w

def computePosition(l_torso, l_neck, q1, q):
    p = [None]*len(q)
    
    for index in range(len(q)):
        r1 = quaternionProduct(q1[index], quaternionProduct([0,0,l_torso,0], conjugateQuaternion(q1[index]) ) )
        r = quaternionProduct(q[index], quaternionProduct([0,0,l_neck,0], conjugateQuaternion(q[index]) ) )
        p[index] = [r1[1]+r[1], r1[2]+r[2], r1[3]+r[3]]
    
    return p
"""
###   PART 5 FUNCTIONS
"""

def plotGyroData(time, gyroX, gyroY, gyroZ):
    fig1 = plt.figure(num='Gyroscope Data')
    ax1 = fig1.add_subplot(111)
    ax1.set_ylabel('tri-axial angular rate, deg/s')
    ax1.set_xlabel('time, s')
    ax1.set_title('IMU Gyroscope Data')
    ax1.plot(time,[x * 180/pi for x in gyroX],label='gyro.X',color='r', linewidth=0.3)
    ax1.plot(time,[y * 180/pi for y in gyroY],label='gyro.Y',color='b', linewidth=0.3)
    ax1.plot(time,[z * 180/pi for z in gyroZ],label='gyro.Z',color='g', linewidth=0.3)
    ax1.legend(loc='best')
    fig1.savefig('IMU Gyroscope Data',dpi=1000)


def plotAccelData(time, accelX, accelY, accelZ):
    fig2 = plt.figure(num='Accelerometer Data')
    ax2 = fig2.add_subplot(111)
    ax2.set_ylabel('Tri-axial acceleration in g (m/s^2)')
    ax2.set_xlabel('time, s')
    ax2.set_title('IMU Accelerometer Data')
    ax2.plot(time,accelX,label='accel.X',color='r', linewidth=0.3)
    ax2.plot(time,accelY,label='accel.Y',color='b', linewidth=0.3)
    ax2.plot(time,accelZ,label='accel.Z',color='g', linewidth=0.3)
    ax2.legend(loc='best')
    fig2.savefig('IMU Accelerometer Data',dpi=1000)
    

def plotMagnetData(time, magnetX, magnetY, magnetZ):
    fig3 = plt.figure(num='Magnetometer Data')
    ax3 = fig3.add_subplot(111)
    ax3.set_ylabel('Tri-axial magnetometer\nflux readings in Gauss (G)')
    ax3.set_xlabel('time, s')
    ax3.set_title('IMU Magnetometer Data')
    ax3.plot(time,magnetX,label='magnet.X',color='r', linewidth=0.3)
    ax3.plot(time,magnetY,label='magnet.Y',color='b', linewidth=0.3)
    ax3.plot(time,magnetZ,label='magnet.Z',color='g', linewidth=0.3)
    ax3.legend(loc='best')
    fig3.savefig('IMU Magnetometer Data',dpi=1000,bbox_inches='tight')


def plotOrientationEulerAnglesFromGyroOnly(time,headings, attitudes, banks):
#    plt.clf()
    fig4 = plt.figure(num='Orientation from Gyro Only')
    ax4 = fig4.add_subplot(111)
    ax4.set_ylabel('Orientation (degs)')
    ax4.set_xlabel('time (s)')
    ax4.set_title('Orientation Euler Angle Approximation from Gyroscope Data Only')
    ax4.plot(time,[x * 180/pi for x in headings], label='Pitch (Y)', color='g', linewidth=0.3)
    ax4.plot(time,[x * 180/pi for x in attitudes], label='Yaw (Z)', color='r', linewidth=0.3)
    ax4.plot(time,[x * 180/pi for x in banks], label='Roll (X)', color='b', linewidth=0.3)
    ax4.legend(loc='best')
    fig4.savefig('Orientation from Gyroscope data only V2', dpi=1000, bbox_inches='tight')
    

def plotTiltCorrectedOrientationEulerAngles(time,yaws, pitches, rolls):
    fig5 = plt.figure(num='Tilt Corrected Orientation')
    ax5 = fig5.add_subplot(111)
    ax5.set_ylabel('Orientation (degs)')
    ax5.set_xlabel('time (s)')
    ax5.set_title('Orientation Euler Angle Approximation with Tilt Correction')
    ax5.plot(time,[x * 180/pi for x in yaws], label='Pitch (Y)', color='g', linewidth=0.3)
    ax5.plot(time,[x * 180/pi for x in pitches], label='Yaw (Z)', color='r', linewidth=0.3)
    ax5.plot(time,[x * 180/pi for x in rolls], label='Roll (X)', color='b', linewidth=0.3)
    ax5.legend(loc='best')
    fig5.savefig('Orientation with Tilt Correction', dpi=1000, bbox_inches='tight')
    
def plotYawTiltCorrectedOrientationEulerAngles(time,yaws, pitches, rolls):
    fig6 = plt.figure(num='Drift Corrected Orientation')
    ax6 = fig6.add_subplot(111)
    ax6.set_ylabel('Orientation (degs)')
    ax6.set_xlabel('time (s)')
    ax6.set_title('Orientation Euler Angle Approximation with Yaw and Tilt Correction')
    ax6.plot(time,[x * 180/pi for x in yaws], label='Pitch (Y)', color='g', linewidth=0.3)
    ax6.plot(time,[x * 180/pi for x in pitches], label='Yaw (Z)', color='r', linewidth=0.3)
    ax6.plot(time,[x * 180/pi for x in rolls], label='Roll (X)', color='b', linewidth=0.3)
    ax6.legend(loc='best')
    fig6.savefig('Orientation with Yaw and Tilt Correction', dpi=1000, bbox_inches='tight')


def Plot3D_data_gen(frame, yaws, pitches, rolls, ax, frameSkip):
    time  = frame*frameSkip
    cy = cos(yaws[time])
    sy = sin(yaws[time])
    cx = cos(pitches[time])
    sx = sin(pitches[time])
    cz = cos(rolls[time])
    sz = sin(rolls[time])
    ax.cla()
    ax.quiver(0,0,0, cy*cx, sz*sy*cx+cz*sx, -cz*sy*cx+sz*sx, pivot="middle", color="blue") # Z axis
    ax.quiver(0,0,0, -cy*sx, -sz*sy*sx+cz*cx, cz*sy*sx+sz*cx, pivot="middle", color="green") # Y axis
    ax.quiver(0,0,0, sy, -sz*cy, cz*cy, pivot="middle", color="red") # X axis
    
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=30, azim=60)
    
def Plot3D_data_gen_Position(frame, yaws, pitches, rolls, position, ax, frameSkip):
    time  = frame*frameSkip
    py = position[time][0]
    pz = position[time][1]
    px = position[time][2]
    
    cy = cos(yaws[time])
    sy = sin(yaws[time])
    cx = cos(pitches[time])
    sx = sin(pitches[time])
    cz = cos(rolls[time])
    sz = sin(rolls[time])
    ax.cla()
    ax.quiver(px,py,pz, cy*cx,  sz*sy*cx+cz*sx,  -cz*sy*cx+sz*sx, pivot="middle", color="blue") # Z axis
    ax.quiver(px,py,pz, -cy*sx, -sz*sy*sx+cz*cx, cz*sy*sx+sz*cx,  pivot="middle", color="green") # Y axis
    ax.quiver(px,py,pz, sy,     -sz*cy,          cz*cy,           pivot="middle", color="red") # X axis
    
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=30, azim=60)

    


"""
time,
gyroscope.X, gyroscope.Y, gyroscope.Z, 
accelerometer.X, accelerometer.Y, accelerometer.Z, 
magnetometer.X, magnetometer.Y, magnetometer.Z 

use Right hand coordinates
Z is 'up' in the global frame of reference
Z = yaw
Y = pitch
X = roll
"""
IMUData = getIMUDataFromCSV("IMUData.csv")
convertDegreesToRads(IMUData)
#normaliseAccelerometerValues(IMUData)
#normaliseMagnetometerValues(IMUData)
time, gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magnetX, magnetY, magnetZ = separateIMUData(IMUData)

frameSkip = 45  # animation plots only plot every (frameSkip) dataPoints. A higher number is better for low spec PCs

print("#########################################")
print("### gvch48 Virtual Reality Assignment ###")
print("#########################################")
print("")
print("Answer questions with y for yes, or any other key for no")
print("questions starting with a * require a yes confirmation to progress")
print("2D graphs are saved to file, 3D plots are not")
print("You must close the window a graph/plot appears before the next prompt will appear")
print("")

"""   PART 1   """
if(str(input("  Create graphs of input data?  ")).lower() == 'y'):
    plotGyroData(time, gyroX,gyroY,gyroZ)
    plotAccelData(time, accelX, accelY, accelZ)
    plotMagnetData(time, magnetX, magnetY, magnetZ)
    plt.show()

"""   Part 2     """
print("")
if(str(input("* Compute orientation from Gyroscope data?  ")).lower() == 'y'):
    vThetaQuats = computeVThetaQuats(time, gyroX, gyroY, gyroZ)
    gyroOriQuats = orientationQuatsFromGyroOnlyVersion2(vThetaQuats, time)
    yaws,pitches,rolls = convertOrientationQuatsToEulerAngles(gyroOriQuats)
    
    if(str(input("  Plot 2D graph of orientation?  ")).lower() == 'y'):
        plotOrientationEulerAnglesFromGyroOnly(time, yaws,pitches,rolls)
        plt.show()
        
    if(str(input("  Create Animated Plot of Orientation?  ")).lower() == 'y'):
        fig7 = plt.figure(num='Animated plot of Orientation from Gyro Data only')
        ax7 = fig7.add_subplot(111, projection='3d')
        ani = animation.FuncAnimation(fig7,Plot3D_data_gen, len(yaws)//frameSkip,fargs=(yaws,pitches,rolls, ax7, frameSkip), interval=frameSkip*(1000/256), blit=False)
        #ani.save('gyroOnlyOrientation.mp4', writer='pillow', extra_args=['-vcodec', 'libx264'])
        plt.show()
        #half speed
    if(str(input("  Play animated Plot at half-speed?  ")).lower() == 'y'):
        fig7B = plt.figure(num='Animated plot of Orientation from Gyro Data only at half speed')
        ax7B = fig7B.add_subplot(111, projection='3d')
        ani = animation.FuncAnimation(fig7B,Plot3D_data_gen, 2*len(yaws)//frameSkip, fargs=(yaws,pitches,rolls, ax7B, frameSkip//2), interval=frameSkip*(1000/256), blit=False)
        plt.show()
    
    """    PART 3    """
    print("")
    if(str(input("* Compute orientation with Tilt correction?  ")).lower() == 'y'):
        averagedAccelX = averageOverNSamples(accelX, 30)
        averagedAccelY = averageOverNSamples(accelY, 30)
        averagedAccelZ = averageOverNSamples(accelZ, 30)
        averagedMagnetX = averageOverNSamples(magnetX, 20)
        averagedMagnetY = averageOverNSamples(magnetY, 20)
        averagedMagnetZ = averageOverNSamples(magnetZ, 20)
        
        normalisedAveragedAccelX, normalisedAveragedAccelY, normalisedAveragedAccelZ = normaliseDataValues(averagedAccelX,averagedAccelY,averagedAccelZ)
        normalisedAveragedMagnetX, normalisedAveragedMagnetY, normalisedAveragedMagnetZ = normaliseDataValues(averagedMagnetX, averagedMagnetY, averagedMagnetZ)
        
        tiltCorrectedOrientationQuats = orientationQuatsWithTiltCorrection(0.1, gyroOriQuats, vThetaQuats, normalisedAveragedAccelX, normalisedAveragedAccelY, normalisedAveragedAccelZ)
        tYaws, tPitches, tRolls = convertOrientationQuatsToEulerAngles(tiltCorrectedOrientationQuats)
        
        if(str(input("  Plot 2D graph of tilt-corrected orientation?  ")).lower() == 'y'):
            plotTiltCorrectedOrientationEulerAngles(time,tYaws, tPitches, tRolls)
            plt.show()
            
        if(str(input("  Create Animated Plot of Tilt-Corrected Orientation?  ")).lower() == 'y'):
            fig8 = plt.figure(num='Animated plot of Tilt-Corrected Orientation')
            ax8 = fig8.add_subplot(111, projection='3d')
            ani = animation.FuncAnimation(fig8,Plot3D_data_gen, len(yaws)//frameSkip,fargs=(tYaws,tPitches,tRolls, ax8, frameSkip), interval=frameSkip*(1000/256), blit=False)
            plt.show()
        #half speed
        if(str(input("  Play animated Plot at half-speed?  ")).lower() == 'y'):
            fig8B = plt.figure(num='Animated plot of Tilt-Corrected Orientation at half-speed')
            ax8B = fig8B.add_subplot(111, projection='3d')
            ani = animation.FuncAnimation(fig8B,Plot3D_data_gen, 2*len(yaws)//frameSkip, fargs=(tYaws,tPitches,tRolls, ax8B, frameSkip//2 ), interval=frameSkip*(1000/256), blit=False)
            plt.show()

    
    
        """     PART 4    """
        print("")
        if(str(input("* Compute orientation with yaw and tilt correction?  ")).lower() == 'y'):
            yawTiltCompensatedOrientationQuats = orientationQuatsWithYawTiltCorrection(0.03, tiltCorrectedOrientationQuats, vThetaQuats, normalisedAveragedMagnetX, normalisedAveragedMagnetY, normalisedAveragedMagnetZ)
            dYaws, dPitches, dRolls = convertOrientationQuatsToEulerAngles(yawTiltCompensatedOrientationQuats)
            
            if(str(input("  Plot 2D graph of drift-corrected orientation?  ")).lower() == 'y'):
                plotYawTiltCorrectedOrientationEulerAngles(time, dYaws, dPitches, dRolls)
                plt.show()
                
            if(str(input("  Create Animated Plot of Drift-Corrected Orientation?  ")).lower() == 'y'):
                fig9 = plt.figure(num='Animated Plot of Drift-Corrected Orientation')
                ax9 = fig9.add_subplot(111, projection='3d')
                ani = animation.FuncAnimation(fig9,Plot3D_data_gen, len(dYaws)//frameSkip,fargs=(dYaws,dPitches,dRolls, ax9, frameSkip), interval=frameSkip*(1000/256), blit=False)
                plt.show()
            #half speed
            if(str(input("  Play animated Plot at half-speed?  ")).lower() == 'y'):
                fig9B = plt.figure(num='Animated Plot of Drift-Corrected Orientation at half speed')
                ax9B = fig9B.add_subplot(111, projection='3d')
                ani = animation.FuncAnimation(fig9B,Plot3D_data_gen, 2*len(dYaws)//frameSkip, fargs=(dYaws,dPitches,dRolls, ax9B, (frameSkip//2)), interval=frameSkip*(1000/256), blit=False)
                plt.show()

            
                
            """    PART 6   """
            print("")
            if(str(input("* Compute Position From Orientation Data?  ")).lower() == 'y'):
                linearAccelX = computeLinearAcceleration(accelX, averagedAccelX)
                linearAccelY = computeLinearAcceleration(accelY, averagedAccelY)
                linearAccelZ = computeLinearAcceleration(accelZ, averagedAccelZ)
                
                integratedLinearAccelX = integrateLinearAcceleration(time, linearAccelX)
                integratedLinearAccelY = integrateLinearAcceleration(time, linearAccelY)
                integratedLinearAccelZ = integrateLinearAcceleration(time, linearAccelZ)
                
                positionVThetaQuats = computeVThetaQuats(time, integratedLinearAccelX, integratedLinearAccelY, integratedLinearAccelZ )
                q1 = orientationQuatsFromGyroOnlyVersion2(positionVThetaQuats, time)
                
                l_torso = 0.5
                l_neck = 0.2
                position = computePosition(l_torso, l_neck, q1, yawTiltCompensatedOrientationQuats)
                
                if(str(input("  Create Animated Plot of position?  ")).lower() == 'y'):
                    fig10 = plt.figure(num='Positional Tracking')
                    ax10 = fig10.add_subplot(111, projection='3d')
                    anim = animation.FuncAnimation(fig10,Plot3D_data_gen_Position, len(dYaws)//frameSkip,fargs=(dYaws,dPitches,dRolls,position, ax10, frameSkip), interval=frameSkip*(1000/256), blit=False)
                    plt.show()
                #half speed
                if(str(input("  Play animated Plot at half-speed?  ")).lower() == 'y'):
                    fig10B = plt.figure(num='Positional Tracking at half speed')
                    ax10B = fig10B.add_subplot(111, projection='3d')
                    anim = animation.FuncAnimation(fig10B,Plot3D_data_gen_Position, 2*len(dYaws)//frameSkip,fargs=(dYaws,dPitches,dRolls,position, ax10, frameSkip/2), interval= frameSkip*(1000/256), blit=False)
                    plt.show()
                
                
                
            print("")
            if(str(input("  Print Final Yaw, Pitch and Roll angles before and after drift correction?  ")).lower() == 'y'):
                print("yaws before and after")
                print(yaws[6958],tYaws[6958], dYaws[6958])
                print("pitches before and after")
                print(pitches[6958], tPitches[6958], dPitches[6958])
                print("rolls before and after")
                print(rolls[6958], tRolls[6958], dRolls[6958])
                print("")

input("Press enter to close.  ")
