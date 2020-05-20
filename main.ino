#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include "MadgwickAHRS.h" // Library from https://github.com/arduino-libraries/MadgwickAHRS/

// TODO: When button is pushed, reset yaw to 0 (or send it in a separate characteristic)

unsigned long lastTime;
unsigned long samplePeriod = 1000000/sampleFreqDef;
float q[4] = {1, 0, 0, 0};
Madgwick madgwick;

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
    lastTime = micros();
}

void loop() {
    if (!central.connected()) {
        central = BLE.central();
    }

    if (micros() - lastTime >= samplePeriod) {
        lastTime += samplePeriod;
        float ax, ay, az, gx, gy, gz/*, mx, my, mz*/;
        IMU.readAcceleration(ax, ay, az);
        // IMU.readMagneticField(mx, my, mz);
        IMU.readGyroscope(gx, gy, gz);
        madgwick.updateIMU(-gx, -gy, -gz, ax, ay, az);
        q[0] = madgwick.q0;
        q[1] = madgwick.q1;
        q[2] = madgwick.q2;
        q[3] = madgwick.q3;
        memcpy(message, q, messageSize);
        imuCharacteristic.writeValue(message, messageSize);
    }
}
