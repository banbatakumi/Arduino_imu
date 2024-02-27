#include <Arduino.h>

#include "MPU6050_6Axis_MotionApps612.h"
#include "fast_digitalwrite.h"

#define CHANGE_TO_DEG 57.2957795131

const uint8_t LED_RED = 16;
const uint8_t LED_GREEN = 17;

MPU6050 mpu;

// MPU control/status vars
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup() {
      Serial.begin(115200);  // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200

      pinMode(LED_GREEN, OUTPUT);
      pinMode(LED_RED, OUTPUT);
      High(LED_GREEN);

      // IMU
      Wire.begin();
      Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

      mpu.initialize();

      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXAccelOffset(-7823);
      mpu.setYAccelOffset(-1292);
      mpu.setZAccelOffset(1180);
      mpu.setXGyroOffset(7);
      mpu.setYGyroOffset(44);
      mpu.setZGyroOffset(14);

      if (devStatus == 0) {
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();

            mpu.setDMPEnabled(true);

            packetSize = mpu.dmpGetFIFOPacketSize();

            Low(LED_GREEN);
      } else {
            High(LED_RED);
      }
}

void loop() {
      while (1) {                                           // 呼び出しのオーバーヘッド節減
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            }

            int16_t yaw = ypr[0] * CHANGE_TO_DEG;
            uint8_t yaw_plus = yaw >= 0 ? yaw : 0;
            uint8_t yaw_minus = yaw < 0 ? yaw * -1 : 0;

            // UART送信
            Serial.write(0xFF);
            Serial.write(yaw_plus);
            Serial.write(yaw_minus);
            Serial.write(0xAA);
      }
}