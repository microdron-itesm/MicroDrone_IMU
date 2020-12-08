#include <queue>
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <common/mavlink.h>

#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"


MPU9250 imu(Wire, 0x68);
volatile ssize_t cts = 0;
volatile bool imuReady = false;

const float radsToDegs = (180.0f / M_PI);

bool calibrated = false;

const bool enableMotionCal = true;
const bool enableWebViewer = false;

std::queue<mavlink_message_t> msgBuffer;
uint8_t msgByteBuffer[MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t)];
size_t currentByte = 0;
size_t msgLen = 0;

unsigned long lastHb = 0;

float acc[3];
float gyro[3];
float mag[3];

float euler_angles[3];

unsigned long lastImuTime = 0;

Madgwick filter;

void initIMUValues(){
  memset(acc, 0, sizeof(acc));
  memset(gyro, 0, sizeof(gyro));
  memset(mag, 0, sizeof(mag));

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
  //while(!SerialUSB);
  SerialUSB.println("DT,Roll,Pitch,Yaw");
  //SerialUSB.println("hello there");

  memset(msgByteBuffer, 0, sizeof(msgByteBuffer));

  int status = imu.begin();

  //imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);

  initIMUValues();

  pinMode(13, INPUT);
  attachInterrupt(digitalPinToInterrupt(13), picInterrupt, FALLING);

  status = imu.calibrateGyro();

  if(status < 0){
    //Die
    while(1);
  }


  status = imu.calibrateAccel();
  if(status < 0){
    //Die
    while(1);
  }
  
  imu.setMagCalX(-10.93f, 0.954f);
  imu.setMagCalY(163.88f, 1.2f);
  imu.setMagCalZ(61.23f, 0.902f);

  imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imu.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  imu.setSrd(0); //Use 100hz update rate
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

  lastHb = millis();

  filter.begin(512);
}

void loop() {
  mavlink_message_t msg;

  if(micros() - lastImuTime > 1953.125){ //500hz
    imu.readSensor();
    imuReady = false;

    acc[0] = imu.getAccelX_mss();//m/s/s
    acc[1] = imu.getAccelY_mss();
    acc[2] = imu.getAccelZ_mss();

    gyro[0] = imu.getGyroX_rads() * RAD_TO_DEG;//rads
    gyro[1] = imu.getGyroY_rads() * RAD_TO_DEG;
    gyro[2] = imu.getGyroZ_rads() * RAD_TO_DEG;

    mag[0] = imu.getMagX_uT();//uT
    mag[1] = imu.getMagY_uT();
    mag[2] = imu.getMagZ_uT();

    filter.update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
    // MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
    //filter.updateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);

    mavlink_msg_attitude_quaternion_pack(1, MAV_COMP_ID_IMU, &msg, millis(), filter.q0, filter.q1, filter.q2, filter.q3, gyro[0] / RAD_TO_DEG, gyro[1] / RAD_TO_DEG, gyro[2] / RAD_TO_DEG);

    // mavlink_msg_attitude_quaternion_pack(1, MAV_COMP_ID_IMU, &msg, millis(), q0, q1, q2, q3, gyro[0] / RAD_TO_DEG, gyro[1] / RAD_TO_DEG, gyro[2] / RAD_TO_DEG);
    msgBuffer.emplace(msg);

    if(enableMotionCal){
      //MotionCal
      SerialUSB.print("Raw:");
      SerialUSB.print(int(acc[0]*8192));
      SerialUSB.print(',');
      SerialUSB.print(int(acc[1]*8192));
      SerialUSB.print(',');
      SerialUSB.print(int(acc[2]*8192));
      SerialUSB.print(',');
      SerialUSB.print(int(gyro[0] * 16));
      SerialUSB.print(',');
      SerialUSB.print(int(gyro[1] * 16));
      SerialUSB.print(',');
      SerialUSB.print(int(gyro[2] * 16));
      SerialUSB.print(',');
      SerialUSB.print(int(mag[0] * 10));
      SerialUSB.print(',');
      SerialUSB.print(int(mag[1] * 10));
      SerialUSB.print(',');
      SerialUSB.print(int(mag[2] * 10));
      SerialUSB.println();
    } else if(enableWebViewer){
      float q[] = {filter.q0, filter.q1, filter.q2, filter.q3};
      float roll, pitch, yaw;

      mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);
      SerialUSB.print(("Orientation: "));
      SerialUSB.print((float)yaw * RAD_TO_DEG);
      SerialUSB.print((" "));
      SerialUSB.print((float)pitch * RAD_TO_DEG);
      SerialUSB.print((" "));
      SerialUSB.print((float)roll * RAD_TO_DEG);
      SerialUSB.println((""));

      // SerialUSB.print(("Orientation: "));
      // SerialUSB.print((float)0);
      // SerialUSB.print((", "));
      // SerialUSB.print((float)0);
      // SerialUSB.print((", "));
      // SerialUSB.print((float)0);
      // SerialUSB.println((""));


      // SerialUSB.print(("Quaternion: "));
      // SerialUSB.print((float)filter.q0);
      // SerialUSB.print((", "));
      // SerialUSB.print((float)filter.q1);
      // SerialUSB.print((", "));
      // SerialUSB.print((float)filter.q2);
      // SerialUSB.print((", "));
      // SerialUSB.print((float)filter.q3);
      // SerialUSB.println((""));

      //SerialUSB.println("Calibration: 0, 0, 0, 0");


    } else{
      unsigned long dt = (micros() - lastImuTime) / 1000;
      float q[] = {filter.q0, filter.q1, filter.q2, filter.q3};
      float roll, pitch, yaw;
      mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);
      //SerialUSB.print("DT: ");
      SerialUSB.print(dt);
      SerialUSB.print(',');
      //SerialUSB.print(" Roll: ");
      SerialUSB.print(roll);
      SerialUSB.print(',');
      //SerialUSB.print(" Pitch: ");
      SerialUSB.print(pitch);
      SerialUSB.print(',');
      //SerialUSB.print(" Yaw: ");
      SerialUSB.print(yaw);
      SerialUSB.println(',');
    }

    lastImuTime = micros();
  }  

  if(millis() - lastHb > 500){ //Send a hb every second
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