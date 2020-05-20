import time
import uuid
import socket
import Adafruit_BluefruitLE

BLE_SERVICE_UUID = uuid.UUID('0f958eb9-b8bb-40e3-91b1-54281cabe755')
IMU_CHAR_UUID    = uuid.UUID('06d66869-9fc1-4141-970d-dd5f6088723a')
ble = Adafruit_BluefruitLE.get_provider()

client = None

def receivedImu(data):
    client.send(data)
    # print(data)

def readBle():
    global client

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
        device.discover([BLE_SERVICE_UUID], [IMU_CHAR_UUID])

        bleService = device.find_service(BLE_SERVICE_UUID)
        imuChar = bleService.find_characteristic(IMU_CHAR_UUID)

        print("Waiting for Fusion 360 client...")
        sock = socket.socket()
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("localhost", 5000))
        sock.listen()
        client, address = sock.accept()
        print("Socket client connected")
        client.send(b'\xc5\xced?\xfeGB\xbe\x8b\x9af\xba\x7f\x13\xd0\xbe')

        print('Subscribing to characteristic changes...')
        imuChar.start_notify(receivedImu)

        while True:
            time.sleep(3600)
    finally:
        device.disconnect()


ble.initialize()
ble.run_mainloop_with(readBle)
