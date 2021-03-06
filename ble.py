#!/usr/bin/env python
import time
import asyncio

# pip install bleak
from bleak import discover, BleakClient

BLE_SERVICE_UUID       = '0f958eb9-b8bb-40e3-91b1-54281cabe755'
ROTATE_CHAR_UUID       = '06d66869-9fc1-4141-970d-dd5f6088723a'
START_ROTATE_CHAR_UUID = 'd9acf2e8-0f26-4707-94eb-091afc18e952'
VELOCITY_CHAR          = '4c3a0eec-d518-4e1e-a8c3-664111eb4d47'
DBL_CLICK_CHAR         = '343056fa-14a3-4b67-9b05-37045461ebd2'

shutdownWriter = None
startRotateWriter = None
rotateWriter = None
velocityWriter = None
dblClickWriter = None

def shutdownConnection(reader, writer):
    global shutdownWriter
    shutdownWriter = writer

def startRotateConnection(reader, writer):
    print("Socket client connected")
    global startRotateWriter
    startRotateWriter = writer

def rotateConnection(reader, writer):
    global rotateWriter
    rotateWriter = writer

def velocityConnection(reader, writer):
    global velocityWriter
    velocityWriter = writer

def dblClickConnection(reader, writer):
    global dblClickWriter
    dblClickWriter = writer

def startRotation(sender, data):
    # print(data)
    if startRotateWriter:
        startRotateWriter.write(bytes(data))

def receivedRotation(sender, data):
    # print(data)
    if rotateWriter:
        rotateWriter.write(bytes(data))

def receivedVelocity(sender, data):
    # print(data)
    if velocityWriter:
        velocityWriter.write(bytes(data))

def receivedDblClick(sender, data):
    # print(data)
    if dblClickWriter:
        dblClickWriter.write(bytes(data))

async def run(loop):
    print('Searching for device...')
    address = None
    devices = await discover()
    for d in devices:
        if 'uuids' in d.metadata:
            if BLE_SERVICE_UUID in d.metadata['uuids']:
                address = d.address
    if address is None:
        print("Arduino BLE device not found")
        return
    
    print('Connecting to device...')
    async with BleakClient(address, loop=loop) as client:
        print('Subscribing to characteristic changes...')
        await client.start_notify(START_ROTATE_CHAR_UUID, startRotation)
        await client.start_notify(ROTATE_CHAR_UUID, receivedRotation)
        await client.start_notify(VELOCITY_CHAR, receivedVelocity)
        await client.start_notify(DBL_CLICK_CHAR, receivedDblClick)
        
        print("Waiting for Fusion 360 client...")
        await asyncio.start_server(shutdownConnection,    'localhost', 4999, loop=loop)
        await asyncio.start_server(startRotateConnection, 'localhost', 5001, loop=loop)
        await asyncio.start_server(rotateConnection,      'localhost', 5000, loop=loop)
        await asyncio.start_server(velocityConnection,    'localhost', 5002, loop=loop)
        await asyncio.start_server(dblClickConnection,    'localhost', 5003, loop=loop)

        while True:
            if not startRotateWriter is None:
                await startRotateWriter.drain()
            if not rotateWriter is None:
                await rotateWriter.drain()
            if not velocityWriter is None:
                await velocityWriter.drain()
            if not dblClickWriter is None:
                await dblClickWriter.drain()
            await asyncio.sleep(0.005, loop=loop)

# notify client that the server will go down
async def cleanup():
    if not shutdownWriter is None:
        shutdownWriter.write(b'a')
        await shutdownWriter.drain()


loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(run(loop))
finally:
    loop.run_until_complete(cleanup())
    loop.close()
