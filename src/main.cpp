#include <Arduino.h>

#include "MPU6050_6Axis_MotionApps612.h"

#define LINE_AVERAGE_NUMBER 100
#define LINE_REACTION_VALUE 100
#define MOTOR_RC 0.25

const uint8_t LED_RED = 16;
const uint8_t LED_GREEN = 17;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa;   // [x, y, z]            accel sensor measurements
VectorInt16 gy;   // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;   // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;   // [x, y, z]            gravity vector
float euler[3];   // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;   // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
      mpuInterrupt = true;
}

void imu_get();

int16_t yaw = 0;
uint8_t yaw_plus = 0, yaw_minus = 0;

void setup() {
      Serial.begin(57600);   // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200

      pinMode(A2, OUTPUT);
      pinMode(A3, OUTPUT);

      digitalWrite(LED_GREEN, HIGH);

      // IMU
      //  join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);   // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
#endif

      mpu.initialize();
      if (mpu.testConnection() != true) {
            return;   // 接続失敗
      }

      devStatus = mpu.dmpInitialize();

      if (devStatus != 0) {
            return;   // 初期化失敗
      }

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(-29);
      mpu.setYGyroOffset(7);
      mpu.setZGyroOffset(-14);
      mpu.setXAccelOffset(-3412);
      mpu.setYAccelOffset(340);
      mpu.setZAccelOffset(556);

      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();

      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      digitalWrite(LED_GREEN, LOW);
}

void loop() {
      imu_get();

      Serial.write('H');
      Serial.write(yaw_plus);
      Serial.write(yaw_minus);
      Serial.flush();
}

void imu_get() {
      if (!dmpReady) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {   // Get the Latest packet
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw_plus = 0;
            yaw_minus = 0;
            yaw = ypr[0] * 180 / M_PI;
            yaw_plus = yaw > 0 ? yaw : 0;
            yaw_minus = yaw < 0 ? yaw * -1 : 0;
      }
}