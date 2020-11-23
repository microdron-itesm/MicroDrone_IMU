#include <queue>
#include <Arduino.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include <common/mavlink.h>
#include "state_estimator.h"

MPU9250_DMP imu;
volatile ssize_t cts = 0;
volatile bool imuReady = false;

std::queue<mavlink_message_t> msgBuffer;
uint8_t msgByteBuffer[MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t)];
size_t currentByte = 0;
size_t msgLen = 0;

unsigned long lastHb = 0;

float acc[3];
float gyro[3];
float mag[3];

float q[4];
float euler_angles[3];

unsigned long lastImuTime = 0;

void initIMUValues(){
  memset(acc, 0, sizeof(acc));
  memset(gyro, 0, sizeof(gyro));
  memset(mag, 0, sizeof(mag));

  memset(q, 0, sizeof(q));
  q[0] = 1.0;
  memset(euler_angles, 0, sizeof(euler_angles));
  lastImuTime = micros();
}

void picInterrupt(){
  cts = 3;
}

void imuInterrupt(){
  imuReady = true;
}

//SCALED_IMU
//ATTITUDE_QUATERNION

void setup() {
  pinMode(4, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(4), imuInterrupt, FALLING);

  Serial1.begin(115200);
  SerialUSB.begin(115200);
  while(!SerialUSB);
  SerialUSB.println("hello there");

  memset(msgByteBuffer, 0, sizeof(msgByteBuffer));

  int status = imu.begin();
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setSampleRate(1000); //Use 1000hz update rate
  imu.setCompassSampleRate(100);

  imu.setAccelFSR(4);
  imu.setGyroFSR(1000);
  imu.setLPF(98);

  imu.enableInterrupt();
  imu.setIntLevel(INT_ACTIVE_LOW); //Active_low interrupt
  imu.setIntLatched(INT_LATCHED);

  initIMUValues();

  if(imu.selfTest() != 0x7){
    //Self test failed
    //Die
    for(;;){
        Serial1.write(0xDE); Serial1.write(0xAD);
    }
  }

  pinMode(13, INPUT);
  attachInterrupt(digitalPinToInterrupt(13), picInterrupt, FALLING);

  lastHb = millis();
}

void loop() {
  mavlink_message_t msg;

  if(true){
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    imuReady = false;

    acc[0] = imu.calcAccel(imu.ax);
    acc[1] = imu.calcAccel(imu.ay);
    acc[2] = imu.calcAccel(imu.az);

    SerialUSB.print(acc[0]);

    gyro[0] = imu.calcGyro(imu.gx);
    gyro[1] = imu.calcGyro(imu.gy);
    gyro[2] = imu.calcGyro(imu.gz);

    mag[0] = imu.calcMag(imu.mx);
    mag[1] = imu.calcMag(imu.my);
    mag[2] = imu.calcMag(imu.mz);

    float dt = ((micros() - lastImuTime) / 1000000.0f);

    UpdateState(acc, gyro, mag, q, dt, euler_angles);
    lastImuTime = micros();

    mavlink_msg_attitude_quaternion_pack(1, MAV_COMP_ID_IMU, &msg, millis(), q[0], q[1], q[2], q[3], gyro[0], gyro[1], gyro[2]);
    msgBuffer.emplace(msg);
    
    SerialUSB.print("Roll: ");
    SerialUSB.print(euler_angles[0]);
    SerialUSB.print(" Pitch: ");
    SerialUSB.print(euler_angles[1]);
    SerialUSB.print(" Yaw: ");
    SerialUSB.println(euler_angles[2]);
  }  

  if(millis() - lastHb > 1000){ //Send a hb every second
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_IMU, &msg, 0,0,0,0,1);
    msgBuffer.emplace(msg);
    
    lastHb = millis();
  }

  if(msgBuffer.size() > 10){
    while(!msgBuffer.empty()) msgBuffer.pop();
  }
  

  if(cts > 0 && !msgBuffer.empty()){
    if(currentByte == msgLen){
      msgLen = mavlink_msg_to_send_buffer(msgByteBuffer, &msgBuffer.front());
      msgBuffer.pop();
      currentByte = 0;
    }

    Serial1.write(msgByteBuffer[currentByte++]);
    cts--;
  }
}