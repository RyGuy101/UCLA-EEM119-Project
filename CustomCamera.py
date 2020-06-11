import adsk.core, adsk.fusion, traceback
import time
import uuid
import struct
import socket
import math

# Run this command in a terminal to see where your python packages are stored:
# python -c "exec(\"import sys\nfor p in sys.path:\n  if 'site-packages' in p:\n    print(p)\")"
import sys
package_path = "/Users/ryannemiroff/miniconda3/lib/python3.7/site-packages"
sys.path.insert(0, package_path)

# pip install pyquaternion
# pip install pyobjc # For MacOS

from pyquaternion import Quaternion
import numpy as np

TRANSLATION_SCALE = 0.2
VELOCITY_THRESHOLD = 0.2 * TRANSLATION_SCALE # 0.2 radian threshold for tilt controls

view = None

refTargetPos = None
refEyePos = None
lastDisplacementTime = None

refDeviceQuat = None
refCameraNormalizedVec = None
refCamUpVec = None

def savePosReference():
    global refTargetPos
    global refEyePos

    refTargetPos = view.camera.target
    refEyePos = view.camera.eye


def getCameraVectors():
    vec = view.camera.target.vectorTo(view.camera.eye)
    vec.normalize()
    cameraNormalizedVec = np.array((vec.x, vec.y, vec.z))

    upVector = view.camera.upVector
    # stolen plane projection code:
    u = np.array((upVector.x, upVector.y, upVector.z))        
    n = cameraNormalizedVec
    proj_of_u_on_n = (np.dot(u, n))*n
    upVec = u - proj_of_u_on_n
    upVec /= np.linalg.norm(upVec)

    return (cameraNormalizedVec, upVec)


def saveQuatReference(deviceQuat):
    global refDeviceQuat
    global refCameraNormalizedVec
    global refCamUpVec

    refDeviceQuat = deviceQuat

    refCameraNormalizedVec, refUpVec = getCameraVectors()
    refCamUpVec = refCameraNormalizedVec + refUpVec


def getBasisMatrix(refDeviceYaw):
    cameraNormalizedVec, upVec = getCameraVectors()
    temp_basis_x = cameraNormalizedVec # These axes should agree with how the Arduino axes are arranged
    temp_basis_z = -upVec
    temp_basis_y = np.cross(temp_basis_z, temp_basis_x)
    temp_basis = np.column_stack((temp_basis_x, temp_basis_y, temp_basis_z))

    # effectively pretend each rotation starts at 0 yaw to alleviate the effects of yaw drift
    basis_x_in_temp_basis = np.array((math.cos(-refDeviceYaw), math.sin(-refDeviceYaw), 0.0))
    basis_x = np.dot(temp_basis, basis_x_in_temp_basis)
    basis_z = temp_basis_z
    basis_y = np.cross(basis_z, basis_x)
    return np.column_stack((basis_x, basis_y, basis_z))


def translate(deviceDisplacement, basis):
    global refTargetPos
    global refEyePos

    camera = view.camera

    r = camera.target.distanceTo(camera.eye)

    # deviceDisplacement = np.array(deviceDisplacement)
    deviceDisplacement[0] = 0 # ignore x direction (perpendicular to computer screen)
    deviceDisplacement *= r

    eye = np.linalg.solve(basis, np.array((camera.eye.x, camera.eye.y, camera.eye.z)))
    target = np.linalg.solve(basis, np.array((camera.target.x, camera.target.y, camera.target.z)))
    eye -= deviceDisplacement # move camera in opposite direction of desired object movement
    target -= deviceDisplacement
    eye = np.dot(basis, eye)
    target = np.dot(basis, target)
    camera.eye = adsk.core.Point3D.create(eye[0], eye[1], eye[2])
    camera.target = adsk.core.Point3D.create(target[0], target[1], target[2])

    camera.isSmoothTransition = False
    view.camera = camera


def velocityToDisplacement(vh, vv):
    global lastDisplacementTime
    savePosReference()
    disp = np.array((0.0, 0.0, 0.0))
    if not lastDisplacementTime is None:
        dt = time.time() - lastDisplacementTime
        disp[1] = vh
        disp[2] = vv
        disp *= TRANSLATION_SCALE
        isMoving = np.abs(disp) > VELOCITY_THRESHOLD
        for i in range(len(disp)):
            if isMoving[i]:
                disp[i] -= VELOCITY_THRESHOLD * np.sign(disp[i])
            else:
                disp[i] = 0
        disp *= dt
    lastDisplacementTime = time.time()
    return disp


def rotate(deviceQuat, basis):
    global refDeviceQuat
    global refCameraNormalizedVec
    global refCamUpVec

    camera = view.camera

    q = deviceQuat * refDeviceQuat.inverse
    q = Quaternion(axis=np.dot(basis, q.axis), angle=q.angle)
    qinv = q.inverse
    # Camera should be rotated opposite to how the user wants to rotate the object:
    rotatedCameraVec = qinv * Quaternion(vector=refCameraNormalizedVec) * q
    rotatedCamUpVec = qinv * Quaternion(vector=refCamUpVec) * q

    rotatedCameraVec = rotatedCameraVec.vector
    rotatedCamUpVec = rotatedCamUpVec.vector

    refDeviceQuat = deviceQuat
    refCameraNormalizedVec = rotatedCameraVec
    refCamUpVec = rotatedCamUpVec

    r = camera.target.distanceTo(camera.eye)
    newEye = r * rotatedCameraVec
    newEye = adsk.core.Vector3D.create(newEye[0], newEye[1], newEye[2])
    newEye.add(camera.target.asVector())
    camera.eye = newEye.asPoint()

    newUp = rotatedCamUpVec - rotatedCameraVec
    camera.upVector = adsk.core.Vector3D.create(newUp[0], newUp[1], newUp[2])

    camera.isSmoothTransition = False
    view.camera = camera


def updateScreen():
    adsk.doEvents()
    view.refresh()


def connectToSocket(port):
    sock = socket.socket()
    sock.connect(("localhost", port))
    sock.setblocking(False)
    return sock


def getLatestData(socket, dataSize):
    try:
        data = socket.recv(dataSize)
        # print(data)
        # get most recent data
        while True:
            try:
                data = socket.recv(dataSize)
            except:
                break
        return data
    except:
        return None


def run(context):
    global view
    global lastDisplacementTime

    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        view = app.activeViewport

        print("connecting to socket...")
        startRotateSock = connectToSocket(5001)
        rotateSock = connectToSocket(5000)
        velocitySock = connectToSocket(5002)
        # doublePressSock = connectToSocket(5003)
        print("connected")

        # TODO could use this to rotate around component center (would also have to rotate target):
        # design = app.activeProduct
        # comp = design.activeComponent
        # com = comp.getPhysicalProperties().centerOfMass

        quaternionDataSize = 4*4; # quaternion is made of 4 floats
        startRotateDataSize = quaternionDataSize + 4 # quaternion plus yaw float value
        velocityDataSize = 4*2 # two floats (horizontal and vertical)

        refYaw = 0
        rawVh = 0
        rawVv = 0

        while True:
            startRotateData = getLatestData(startRotateSock, startRotateDataSize)
            quatData = None
            velocityData = None
            # doublePressData = None
            if not startRotateData is None:
                q = Quaternion(np.array(struct.unpack('4f', startRotateData[:quaternionDataSize])))
                saveQuatReference(q)
                refYaw = struct.unpack('1f', startRotateData[quaternionDataSize:])[0]
            else:
                quatData = getLatestData(rotateSock, quaternionDataSize)
            if not quatData is None and not refDeviceQuat is None:
                lastDisplacementTime = None
                q = Quaternion(np.array(struct.unpack('4f', quatData)))
                # print(q)
                rotate(q, getBasisMatrix(refYaw))
                updateScreen()
            else:
                velocityData = getLatestData(velocitySock, velocityDataSize)
            if not velocityData is None:
                velocity = struct.unpack('2f', velocityData)
                rawVh = velocity[0]
                rawVv = velocity[1]
                # print(rawVv)
                if rawVh == 0.0 and rawVv == 0.0:
                    lastDisplacementTime = None
                else:
                    disp = velocityToDisplacement(rawVh, rawVv)
                    translate(disp, getBasisMatrix(0))
                    updateScreen()
            else:
            #     doublePressData = getLatestData(doublePressSock, 1)
            # if not doublePressData is None:
            #     camera = view.camera
            #     camera.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
            #     camera.isFitView = True
            #     camera.isSmoothTransition = True
            #     view.camera = camera
            #     updateScreen()
            # else:
                adsk.doEvents()
                try:
                    startRotateSock.send(b'a') # dummy message to check if server is alive
                except:
                    print("Lost connection with BLE process. Stopping...")
                    break

        # Test code
        # saveQuatReference(Quaternion(1, 0, 0, 0))
        # for i in range(1, 361, 2):
        #     rotate(Quaternion(axis=(0, 0, 1), degrees=i), getBasisMatrix())
        #     updateScreen()
        # for i in range(1, 361, 2):
        #     rotate(Quaternion(axis=(0, 1, 0), degrees=i), getBasisMatrix())
        #     updateScreen()
        # for i in range(1, 361, 2):
        #     rotate(Quaternion(axis=(1, 0, 0), degrees=i), getBasisMatrix())
        #     updateScreen()

        # savePosReference()
        # for i in range(1, 101):
        #     translate((0.0, 0.0, i), getBasisMatrix())
        #     updateScreen()
        # for i in range(1, 101):
        #     translate((0.0, i, 0.0), getBasisMatrix())
        #     updateScreen()
        # for i in range(1, 101):
        #     translate((0.0, 0.0, -i), getBasisMatrix())
        #     updateScreen()
        # for i in range(1, 101):
        #     translate((0.0, -i, 0.0), getBasisMatrix())
        #     updateScreen()

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
