#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include "MadgwickAHRS.h" // Library from https://github.com/arduino-libraries/MadgwickAHRS/

#define ROTATE_BUTTON 8 // digital pin number

bool setupOkay = true;

Madgwick madgwick;
float q[4] = {1, 0, 0, 0};
volatile bool rotating = false;
volatile bool initiatedRotation = false;

const size_t quaternionMessageSize = sizeof(float) * 4;
uint8_t quaternionMessage[quaternionMessageSize];
const size_t startRotateMessageSize = quaternionMessageSize + sizeof(float);
uint8_t startRotateMessage[startRotateMessageSize];

BLEService bleService("0f958eb9-b8bb-40e3-91b1-54281cabe755"); // https://www.uuidgenerator.net/
BLECharacteristic rotateChar("06d66869-9fc1-4141-970d-dd5f6088723a", BLERead | BLENotify, quaternionMessageSize);
BLECharacteristic startRotateChar("d9acf2e8-0f26-4707-94eb-091afc18e952", BLERead | BLEIndicate, startRotateMessageSize);
BLEDevice central;

void rotateButtonChange() {
  rotating = !rotating;
  initiatedRotation = false;
}

void setup() {
    Serial.begin(115200);

    pinMode(ROTATE_BUTTON, INPUT_PULLUP);
    rotating = !digitalRead(ROTATE_BUTTON);
    attachInterrupt(digitalPinToInterrupt(ROTATE_BUTTON), rotateButtonChange, CHANGE);
    
    BLE.begin();
    BLE.setLocalName("RyansArduino");
    BLE.setAdvertisedService(bleService);
    bleService.addCharacteristic(rotateChar);
    bleService.addCharacteristic(startRotateChar);
    BLE.addService(bleService);
    BLE.advertise();

    IMU.begin();

    if (IMU.gyroscopeSampleRate() != sampleFreqDef || IMU.accelerationSampleRate() != sampleFreqDef) {
        Serial.println("IMU sample rates are not correct!");
        setupOkay = false;
    }
}

void loop() {
    if (!setupOkay) {
        Serial.println("Something went wrong during setup!");
        delay(1000);
    }

    if (!central.connected()) {
        central = BLE.central();
    }

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        float ax, ay, az, gx, gy, gz/*, mx, my, mz*/;
        IMU.readAcceleration(ax, ay, az);
        // IMU.readMagneticField(mx, my, mz);
        IMU.readGyroscope(gx, gy, gz);
        madgwick.updateIMU(-gx, -gy, -gz, ax, ay, az);

        if (rotating) {
            q[0] = madgwick.q0;
            q[1] = madgwick.q1;
            q[2] = madgwick.q2;
            q[3] = madgwick.q3;

            if (!initiatedRotation) {
                memcpy(startRotateMessage, q, quaternionMessageSize);
                float yaw = madgwick.getYawRadians();
                memcpy(startRotateMessage+quaternionMessageSize, &yaw, sizeof(yaw));
                startRotateChar.writeValue(startRotateMessage, startRotateMessageSize);
                initiatedRotation = true;
                Serial.println("started rotation");
            } else {
                memcpy(quaternionMessage, q, quaternionMessageSize);
                rotateChar.writeValue(quaternionMessage, quaternionMessageSize);
            }
        }
    }
}
