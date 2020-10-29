#include <queue>
#include <Arduino.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include <common/mavlink.h>

MPU9250_DMP imu;
volatile ssize_t cts = 0;

std::queue<mavlink_message_t> msgBuffer;
uint8_t msgByteBuffer[MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t)];
size_t currentByte = 0;
size_t msgLen = 0;

unsigned long lastHb = 0;

void picInterrupt(){
  cts = 3;
}
//SCALED_IMU
//ATTITUDE_QUATERNION
void setup() {
  Serial1.begin(115200);
  SerialUSB.begin(115200);
  memset(msgByteBuffer, 0, sizeof(msgByteBuffer));

  int status = imu.begin();
  imu.setSampleRate(1000); //Use 1000hz update rate
  imu.setLPF(188);
  imu.setCompassSampleRate(100);

  imu.enableInterrupt(0);
  imu.setIntLevel(1); //Active_low interrupt

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO, 200);

  if(imu.selfTest() != 0x7){
    //Self test failed
    //Die
    pinMode(13, OUTPUT);
    for(;;){
        Serial1.write(0xDE); Serial1.write(0xAD);
    }
  }

  //pinMode(4, INPUT);
  //attachInterrupt(4, imuInterrupt, RISING);

  pinMode(13, INPUT);
  attachInterrupt(13, picInterrupt, FALLING);

  lastHb = millis();
}

void loop() {
  //Only update msgs if buffer has space
  if(msgBuffer.size() < 4){
    mavlink_message_t msg;

    if(imu.fifoAvailable() && imu.dmpUpdateFifo() == INV_SUCCESS){
      imu.updateCompass();
      imu.computeCompassHeading();

      //Read all values from imu
      float q1 = imu.calcQuat(imu.qw);
      float q2 = imu.calcQuat(imu.qx);
      float q3 = imu.calcQuat(imu.qy);
      float q4 = imu.calcQuat(imu.qz);

      float accelX = imu.calcAccel(imu.ax);
      float accelY = imu.calcAccel(imu.ay);
      float accelZ = imu.calcAccel(imu.az);
      float gyroX = imu.calcGyro(imu.gx);
      float gyroY = imu.calcGyro(imu.gy);
      float gyroZ = imu.calcGyro(imu.gz);
      float magX = imu.calcMag(imu.mx);
      float magY = imu.calcMag(imu.my);
      float magZ = imu.calcMag(imu.mz);

      mavlink_msg_attitude_quaternion_pack(1, MAV_COMP_ID_IMU, &msg, millis(), q1, q2, q3, q4, gyroX, gyroY, gyroZ);
      msgBuffer.emplace(msg);

      imu.computeEulerAngles();
      SerialUSB.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));

      //mavlink_msg_raw_imu_pack(1, MAV_COMP_ID_IMU, &msg, millis(), accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
      //msgBuffer.emplace(msg);

      SerialUSB.println("Sending");
    }

    if(millis() - lastHb > 1000){ //Send a hb every second
      mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_IMU, &msg, 0,0,0,0,1);
      msgBuffer.emplace(msg);
      
      lastHb = millis();
    }
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