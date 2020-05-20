import adsk.core, adsk.fusion, traceback
import time
import uuid
import struct
import socket

# Run this command in a terminal to see where your python packages are stored:
# python -c "exec(\"import sys\nfor p in sys.path:\n  if 'site-packages' in p:\n    print(p)\")"
import sys
package_path = "/Users/ryannemiroff/miniconda3/lib/python3.7/site-packages"
sys.path.insert(0, package_path + "/Adafruit_BluefruitLE-0.9.10-py3.7.egg")
sys.path.insert(0, package_path)

# pip install pyquaternion
# pip install pyobjc # For MacOS

# git clone https://github.com/adafruit/Adafruit_Python_BluefruitLE.git
# cd Adafruit_Python_BluefruitLE
# python setup.py install
# cd ..
# rm -rf Adafruit_Python_BluefruitLE 

from pyquaternion import Quaternion
import numpy as np

TRANSLATION_SCALE = 0.000005

view = None

refTargetPos = None
refEyePos = None

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
    u = np.array([upVector.x, upVector.y, upVector.z])        
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


def getBasisMatrix():
    cameraNormalizedVec, upVec = getCameraVectors()
    basis_x = cameraNormalizedVec # Vector from target to eye is set to the positive x direction, but this could change.
    basis_z = upVec
    basis_y = np.cross(basis_z, basis_x)
    return np.column_stack((basis_x, basis_y, basis_z))


def translate(deviceDisplacement, basis):
    global refTargetPos
    global refEyePos

    camera = view.camera

    r = camera.target.distanceTo(camera.eye)

    deviceDisplacement = np.array(deviceDisplacement)
    deviceDisplacement[0] = 0 # ignore x direction (perpendicular to computer screen)
    deviceDisplacement *= r * TRANSLATION_SCALE

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


def run(context):
    global view

    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        view = app.activeViewport

        print("connecting to socket...")
        sock = socket.socket()
        sock.connect(("localhost", 5000))
        sock.setblocking(False)
        print("connected")

        # TODO could use this to rotate around component center (would also have to rotate target):
        # design = app.activeProduct
        # comp = design.activeComponent
        # com = comp.getPhysicalProperties().centerOfMass

        control = False
        dataSize = 4*4; # quaternion is made of 4 floats

        while True:
            try:
                data = sock.recv(dataSize)
                # get most recent data
                while True:
                    try:
                        data = sock.recv(dataSize)
                    except:
                        break
                q = Quaternion(np.array(struct.unpack('4f', data)))
                print(q)

                if not control:
                    saveQuatReference(q)
                    control = True
                else:
                    rotate(q, getBasisMatrix())
                    updateScreen()
            except:
                adsk.doEvents()
                try:
                    sock.send(b'a') # dummy message to check if server is alive
                except:
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
