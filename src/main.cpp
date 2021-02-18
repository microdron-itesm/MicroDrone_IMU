#include <queue>
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <common/mavlink.h>

#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"
//#include <SPI.h>
#include <SD.h>

#include "IMU_Fusion.h"


MPU9250 imu(Wire, 0x68);
volatile ssize_t cts = 0;
volatile bool imuReady = false;

const float radsToDegs = (180.0f / M_PI);

bool calibrated = false;

const bool enableMotionCal = false;
const bool enableWebViewer = false;

float m0, m1, m2, m3;

std::queue<mavlink_message_t> msgBuffer;
uint8_t msgByteBuffer[MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t)];
size_t currentByte = 0;
size_t msgLen = 0;

unsigned long lastHb = 0;

IMU_FusionModelClass::ExtU_IMU_Fusion_T ahrsIn;
IMU_FusionModelClass imuFusion;

unsigned long lastImuTime = 0;

File logFile;
String buffer;

void initIMUValues(){
  memset(&ahrsIn, 0, sizeof(ahrsIn));

  imuFusion.initialize();
  lastImuTime = micros();
}

void picInterrupt(){
  cts = 3;
}

void imuInterrupt(){
  imuReady = true;
}

String generateFileName(int num){
  String sNum(num, DEC);
  return String("log_") + sNum + String(".csv");
}

//SCALED_IMU
//ATTITUDE_QUATERNION

void setup() {
  Serial1.begin(115200);
  SerialUSB.begin(115200);

  pinMode(4, INPUT_PULLUP);
  while(!SerialUSB);
  //attachInterrupt(digitalPinToInterrupt(4), imuInterrupt, FALLING);

  //SD Card init
  buffer.reserve(4096);

  while(!SD.begin(38)){
    SerialUSB.println("SD failed!");
    //while(1);
  }

  //SD Card init done
  File root = SD.open("/");
  int fileNum = 0;
  while(SD.exists(generateFileName(fileNum))){
    fileNum++;
  }

  SerialUSB.print("Using: "); SerialUSB.println(generateFileName(fileNum));
  logFile = SD.open(generateFileName(fileNum), FILE_WRITE);

  if(!logFile){
    SerialUSB.println("log open failed");
    while(1);
  }

  //Write csv header to sd card
  logFile.println("Micros,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,M1,M2,M3,M4");

  //while(!SerialUSB);
  //SerialUSB.println("DT,Roll,Pitch,Yaw");
  //SerialUSB.println("hello there");

  memset(msgByteBuffer, 0, sizeof(msgByteBuffer));

  int status = imu.begin();

  //imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);

  initIMUValues();

  pinMode(13, INPUT);
  attachInterrupt(digitalPinToInterrupt(13), picInterrupt, FALLING);

  //status = imu.calibrateGyro();

  if(status < 0){
    //Die
    while(1);
  }


  //status = imu.calibrateAccel();
  if(status < 0){
    //Die
    while(1);
  }
  
  //imu.setMagCalX(-10.93f, 0.954f);
  //imu.setMagCalY(163.88f, 1.2f);
  //imu.setMagCalZ(61.23f, 0.902f);

  imu.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  imu.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  imu.setSrd(0); //Use 100hz update rate
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);

  lastHb = millis();
}

void loop() {
  mavlink_message_t msg, inMsg;
  mavlink_status_t status;
  imu.readSensor();

  if(micros() - lastImuTime > 10000){ //250hz
    imuReady = false;

    ahrsIn.Accel_Raw[0] = imu.getAccelX_mss();//m/s/s
    ahrsIn.Accel_Raw[1] = imu.getAccelY_mss();
    ahrsIn.Accel_Raw[2] = imu.getAccelZ_mss();

    ahrsIn.Gyro_Raw[0] = imu.getGyroX_rads() * RAD_TO_DEG;//rads
    ahrsIn.Gyro_Raw[1] = imu.getGyroY_rads() * RAD_TO_DEG;
    ahrsIn.Gyro_Raw[2] = imu.getGyroZ_rads() * RAD_TO_DEG;

    ahrsIn.Mag_Raw[0] = imu.getMagX_uT();//uT
    ahrsIn.Mag_Raw[1] = imu.getMagY_uT();
    ahrsIn.Mag_Raw[2] = imu.getMagZ_uT();

    // filter.update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
    // MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);

    imuFusion.setExternalInputs(&ahrsIn);
    imuFusion.step();
    auto ahrsOut = imuFusion.getExternalOutputs();

    mavlink_msg_attitude_quaternion_pack(1, MAV_COMP_ID_IMU, &msg, millis(), ahrsOut.Quat[0], ahrsOut.Quat[1], ahrsOut.Quat[2], ahrsOut.Quat[3], ahrsIn.Gyro_Raw[0] / RAD_TO_DEG, ahrsIn.Gyro_Raw[1] / RAD_TO_DEG, ahrsIn.Gyro_Raw[2] / RAD_TO_DEG);

    //float q[] = {filter.q0, filter.q1, filter.q2, filter.q3};
    double roll, pitch, yaw;
    roll = ahrsOut.Euler[1];
    pitch = ahrsOut.Euler[2];
    yaw = ahrsOut.Euler[0];

    buffer += micros(); buffer += ',';
    
    //RPY
    buffer += roll; buffer += ',';
    buffer += pitch; buffer += ',';
    buffer += yaw; buffer += ',';

    //Accel
    buffer += ahrsIn.Accel_Raw[0]; buffer += ',';
    buffer += ahrsIn.Accel_Raw[1]; buffer += ',';
    buffer += ahrsIn.Accel_Raw[2]; buffer += ',';

    //Gyro
    buffer += ahrsIn.Gyro_Raw[0]; buffer += ',';
    buffer += ahrsIn.Gyro_Raw[1]; buffer += ',';
    buffer += ahrsIn.Gyro_Raw[2]; buffer += ',';

    //Mag
    buffer += ahrsIn.Mag_Raw[0]; buffer += ',';
    buffer += ahrsIn.Mag_Raw[1]; buffer += ',';
    buffer += ahrsIn.Mag_Raw[2]; buffer += ',';
    
    buffer += m0; buffer += ',';
    buffer += m1; buffer += ',';
    buffer += m2; buffer += ',';
    buffer += m3; buffer += '\n';

    // mavlink_msg_attitude_quaternion_pack(1, MAV_COMP_ID_IMU, &msg, millis(), q0, q1, q2, q3, gyro[0] / RAD_TO_DEG, gyro[1] / RAD_TO_DEG, gyro[2] / RAD_TO_DEG);
    msgBuffer.emplace(msg);

    if(enableMotionCal){
      //MotionCal
      SerialUSB.print("Raw:");
      SerialUSB.print(int(ahrsIn.Accel_Raw[0]*8192));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Accel_Raw[1]*8192));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Accel_Raw[2]*8192));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Gyro_Raw[0] * 16));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Gyro_Raw[1] * 16));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Gyro_Raw[2] * 16));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Mag_Raw[0] * 10));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Mag_Raw[1] * 10));
      SerialUSB.print(',');
      SerialUSB.print(int(ahrsIn.Mag_Raw[2] * 10));
      SerialUSB.println();
    } else if(enableWebViewer){
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
      //float q[] = {filter.q0, filter.q1, filter.q2, filter.q3};
      float roll, pitch, yaw;
      roll = ahrsOut.Euler[1];
      pitch = ahrsOut.Euler[2];
      yaw = ahrsOut.Euler[0];
      //mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);
      //SerialUSB.print("DT: ");
      // SerialUSB.print(dt);
      // SerialUSB.print(',');
      // //SerialUSB.print(" Roll: ");
      // SerialUSB.print(roll);
      // SerialUSB.print(',');
      // //SerialUSB.print(" Pitch: ");
      // SerialUSB.print(pitch);
      // SerialUSB.print(',');
      // //SerialUSB.print(" Yaw: ");
      // SerialUSB.print(yaw);
      // SerialUSB.println(',');
    }

    lastImuTime = micros();
  }  

  if(millis() - lastHb > 500){ //Send a hb every second
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_IMU, &msg, 0,0,0,0,1);
    msgBuffer.emplace(msg);
    
    lastHb = millis();
  }

  if(msgBuffer.size() > 4){
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

  while(Serial1.available() > 0){
    uint8_t c = Serial1.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &inMsg, &status)){
      switch(inMsg.msgid){
        case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
          mavlink_debug_float_array_t debugArray;
          mavlink_msg_debug_float_array_decode(&inMsg, &debugArray);
          m0 = debugArray.data[0];
          m1 = debugArray.data[1];
          m2 = debugArray.data[2];
          m3 = debugArray.data[3];
          break;
      }
    }
  }

  unsigned int chunkSize = logFile.availableForWrite();
  if (chunkSize && buffer.length() >= chunkSize) {
    //SerialUSB.println("Writing to SD!");
    // write to file and blink LED
    logFile.write(buffer.c_str(), chunkSize);

    // remove written data from buffer
    buffer.remove(0, chunkSize);
  }
}