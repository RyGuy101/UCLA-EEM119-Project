#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include "orientation.h"

#define PI_OVER_180 0.01745329251f

unsigned long lastTime;
unsigned long currentTime;
float q[4] = {1, 0, 0, 0};

const int messageSize = sizeof(float) * 4;
uint8_t message[messageSize];
BLEService bleService("0f958eb9-b8bb-40e3-91b1-54281cabe755"); // https://www.uuidgenerator.net/
BLECharacteristic imuCharacteristic("06d66869-9fc1-4141-970d-dd5f6088723a", BLERead | BLENotify, messageSize);
BLEDevice central;

void setup() {
    Serial.begin(115200);
    
    BLE.begin();
    BLE.setLocalName("RyansArduino");
    BLE.setAdvertisedService(bleService);
    bleService.addCharacteristic(imuCharacteristic);
    BLE.addService(bleService);
    BLE.advertise();

    IMU.begin();
    currentTime = micros();
}

void loop() {
    if (!central.connected()) {
        central = BLE.central();
    }

    if (IMU.gyroscopeAvailable()) {
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        IMU.readAcceleration(ax, ay, az);
        IMU.readMagneticField(mx, my, mz);
        IMU.readGyroscope(gx, gy, gz);
        gx *= PI_OVER_180;
        gy *= PI_OVER_180;
        gz *= PI_OVER_180;
        currentTime = micros();
        MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, q, (currentTime - lastTime) / 1000000.0f);
        lastTime = currentTime;
        memcpy(message, q, messageSize);
        imuCharacteristic.writeValue(message, messageSize);
    }
}
