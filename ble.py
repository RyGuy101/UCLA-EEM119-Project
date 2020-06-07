import time
import uuid
import socket
import Adafruit_BluefruitLE
# Adafruit_BluefruitLE installation steps:
# git clone https://github.com/adafruit/Adafruit_Python_BluefruitLE.git
# cd Adafruit_Python_BluefruitLE
# python setup.py install
# cd ..
# rm -rf Adafruit_Python_BluefruitLE 

BLE_SERVICE_UUID       = uuid.UUID('0f958eb9-b8bb-40e3-91b1-54281cabe755')
ROTATE_CHAR_UUID       = uuid.UUID('06d66869-9fc1-4141-970d-dd5f6088723a')
START_ROTATE_CHAR_UUID = uuid.UUID('d9acf2e8-0f26-4707-94eb-091afc18e952')

ble = Adafruit_BluefruitLE.get_provider()

startRotateClient = None
rotateClient = None

def startRotation(data):
    if startRotateClient:
        startRotateClient.send(data)
    # print("start rotation")
    # print(data)

def receivedRotation(data):
    if rotateClient:
        rotateClient.send(data)
    # print(data)

def makeSocket(port):
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("localhost", port))
    sock.listen()
    return sock

def readBle():
    global startRotateClient
    global rotateClient

    ble.clear_cached_data()
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))
    ble.disconnect_devices([BLE_SERVICE_UUID])

    print('Searching for device...')
    try:
        adapter.start_scan()
        device = ble.find_device(service_uuids=[BLE_SERVICE_UUID], timeout_sec=60)
        if device is None:
            raise RuntimeError('Failed to find device!')
    finally:
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect(timeout_sec=10)

    try:
        print('Discovering services...')
        device.discover([BLE_SERVICE_UUID], [])

        bleService = device.find_service(BLE_SERVICE_UUID)
        rotationChar = bleService.find_characteristic(ROTATE_CHAR_UUID)
        startRotateChar = bleService.find_characteristic(START_ROTATE_CHAR_UUID)

        print('Subscribing to characteristic changes...')
        startRotateChar.start_notify(startRotation)
        rotationChar.start_notify(receivedRotation)

        print("Waiting for Fusion 360 client...")
        startRotateSock = makeSocket(5001)
        rotateSock = makeSocket(5000)
        startRotateClient, address = startRotateSock.accept()
        rotateClient, address = rotateSock.accept()
        print("Socket client connected")

        while True:
            time.sleep(3600)
    finally:
        device.disconnect()


ble.initialize()
ble.run_mainloop_with(readBle)
