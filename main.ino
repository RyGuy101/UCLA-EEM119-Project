#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include "MadgwickAHRS.h" // Library from https://github.com/arduino-libraries/MadgwickAHRS/

#define ROTATE_BUTTON 8 // digital pin number
#define TRANSLATE_BUTTON 2 // digital pin number

bool setupOkay = true;

Madgwick madgwick;
volatile bool rotating = false;
volatile bool initiatedRotation = false;
volatile bool translating = false;
volatile bool endedTranslate = false;

float q[4];
float vel[2]; // for now, roll and pitch
const float stopped[] = {0, 0};

const size_t quaternionMessageSize = sizeof(float) * 4;
uint8_t quaternionMessage[quaternionMessageSize];
const size_t startRotateMessageSize = quaternionMessageSize + sizeof(float);
uint8_t startRotateMessage[startRotateMessageSize];
const size_t velocityMessageSize = sizeof(float) * 2;
uint8_t velocityMessage[velocityMessageSize];
const size_t doubleClickMessageSize = sizeof(float) * 2;
uint8_t doubleClickMessage[doubleClickMessageSize];

unsigned long startTime = 0;
const int doubleClickThresh = 600; // limit of 600 ms to wait for 2nd button click (release)
int doubleClickButton;
int buttonState = 0;

BLEService bleService("0f958eb9-b8bb-40e3-91b1-54281cabe755"); // https://www.uuidgenerator.net/
BLECharacteristic rotateChar("06d66869-9fc1-4141-970d-dd5f6088723a", BLERead | BLENotify, quaternionMessageSize);
BLECharacteristic startRotateChar("d9acf2e8-0f26-4707-94eb-091afc18e952", BLERead | BLEIndicate, startRotateMessageSize);
BLECharacteristic velocityChar("4c3a0eec-d518-4e1e-a8c3-664111eb4d47", BLERead | BLENotify, velocityMessageSize);
BLECharacteristic doubleClickChar("343056fa-14a3-4b67-9b05-37045461ebd2", BLERead | BLENotify, doubleClickMessageSize);

BLEDevice central;

// void rotateButtonChange() {
//   rotating = !rotating;
//   initiatedRotation = false;
// }

// void translateButtonChange() {
//   translating = !translating;
//   endedTranslate = false;
// }

void setup() {
    Serial.begin(115200);

    pinMode(ROTATE_BUTTON, INPUT_PULLUP);
    // rotating = !digitalRead(ROTATE_BUTTON);
    // attachInterrupt(digitalPinToInterrupt(ROTATE_BUTTON), rotateButtonChange, CHANGE);

    pinMode(TRANSLATE_BUTTON, INPUT_PULLUP);
    // translating = !digitalRead(TRANSLATE_BUTTON);
    // attachInterrupt(digitalPinToInterrupt(TRANSLATE_BUTTON), translateButtonChange, CHANGE);
    
    BLE.begin();
    BLE.setLocalName("Arduino");
    BLE.setAdvertisedService(bleService);
    bleService.addCharacteristic(rotateChar);
    bleService.addCharacteristic(startRotateChar);
    bleService.addCharacteristic(velocityChar);
    bleService.addCharacteristic(doubleClickChar);
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

    // reset double click timer
    if (millis() - startTime > doubleClickThresh)
      buttonState = 0;

    // search for double click
    switch (buttonState) {
      case 0: // press 1
        if (!digitalRead(ROTATE_BUTTON) || !digitalRead(TRANSLATE_BUTTON)) {
          doubleClickButton = !digitalRead(ROTATE_BUTTON) ? ROTATE_BUTTON : TRANSLATE_BUTTON;
          startTime = millis();
          buttonState = 1;
        }
        break;
      case 1: // release 1
        if (digitalRead(doubleClickButton))
          buttonState = 2;
        break;
      case 2: // press 2
        if (!digitalRead(doubleClickButton))
          buttonState = 3;
        break;
      case 3: // release 2
        if (digitalRead(doubleClickButton)) {
          // SUCCESS: NOTIFY/SEND DATA
          buttonState = 0;
        }
      default:
        Serial.println("um how did this happen");
        break;
    }

    if (!digitalRead(ROTATE_BUTTON) != rotating) {
        rotating = !rotating;
        initiatedRotation = false;
    }
    if (!digitalRead(TRANSLATE_BUTTON) != translating) {
        translating = !translating;
        endedTranslate = false;
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
        } else if (translating) {
            vel[0] = madgwick.getRollRadians();
            vel[1] = madgwick.getPitchRadians();
            vel[0] -= 3.14159265f; // Arduino is upside down
            if (vel[0] < -3.14159265f) {
                vel[0] += 2*3.14159265f;
            }
            vel[1] += 0.349; // wrist angle is naturally around 20(?) degress - use this as the zero position
            memcpy(velocityMessage, vel, velocityMessageSize);
            velocityChar.writeValue(velocityMessage, velocityMessageSize);
        } else if (!endedTranslate) {
            memcpy(velocityMessage, stopped, velocityMessageSize);
            velocityChar.writeValue(velocityMessage, velocityMessageSize);
            endedTranslate = true;
        }
    }
}
