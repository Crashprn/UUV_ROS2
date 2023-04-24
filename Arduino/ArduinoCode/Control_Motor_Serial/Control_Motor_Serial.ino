#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// Defining number of pins
const uint8_t numberOfPins = 4;

// 1. PWM Pin, 2. IN1 pin, 3. IN2 pin
// Right, Left, Back, Forward
int pinSets[][3] = {{11, 13, 12}, {10, 9, 8}, {5,7,6}, {3, 4, 2}};

// Array for holding motor messages
char receivedChars[numberOfPins * 2];
// Array for holding new motor values
long int newPinStates[numberOfPins];
/// Array for holding current motor values
long int pinState[numberOfPins];
// Deadzone for motor control
int deadzone = 5;


imu::Vector<3> acc;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  // Setting motor control pins to output
  for (auto &&pins : pinSets)
  {
    pinMode(pins[0], OUTPUT);
    pinMode(pins[1], OUTPUT);
    pinMode(pins[2], OUTPUT);
  }
  Serial.begin(115200);

  for (auto &pin : pinState)
  {
    pin = 0;
  }

  // Checking if IMU is connected
  if (!bno.begin()) Serial.print("0;");
  else Serial.print("1;");
  bno.setExtCrystalUse(true);
  
  confirmCycle();
  
  // Calibrating IMU
  uint8_t system, gyro, accel, mag = 0;
  while (system != 3 || gyro !=3 /* || accel !=3*/ || mag!=3)
  {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    Serial.print("Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.print(mag, DEC);
    Serial.print(";");
    confirmCycle();  
  }
  Serial.print("Calibrated;");
  
}

void loop() {
  // Checking if motor data message is available
  if (Serial.available() >= numberOfPins * 2)
  {
    // Receive motor data message
    receiveSerial(numberOfPins * 2);
    /*
    for (char num: receivedChars)
    {
      Serial.print(static_cast<uint8_t>(num)); Serial.print(" , ");
    }
    */    

    // Setting motor values
    controlMotor(newPinStates);

    // Getting IMU data and sending it back
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    sendIMU(quat, acc);
  }

}

// Function for receiving motor data message
void receiveSerial(int byteNumber)
{
  auto length = Serial.readBytes(receivedChars, byteNumber);
  for (auto i = 0; i < byteNumber/2; i++)
  {
    long int newNum = static_cast<uint8_t>(receivedChars[i*2+1]);
    if (receivedChars[i*2] == '-')
    {
      newNum = newNum * -1;
    }
    newPinStates[i] = static_cast<long int>(newNum);
  }
}

// Function for controlling motors
void controlMotor(long int* motorValues)
{
  for (auto i = 0; i < numberOfPins; i++)
  {
    auto motorValue = motorValues[i];
    if (abs(motorValue) < deadzone || (motorValue < 0 && pinState[i] > 0) || (motorValue > 0 && pinState[i] < 0))
    {
      digitalWrite(pinSets[i][1], LOW);
      digitalWrite(pinSets[i][2], LOW);
      pinState[i] = 0;
    }
    else 
    {
      analogWrite(pinSets[i][0], abs(motorValue));

      if (motorValue < 0)
      {
        digitalWrite(pinSets[i][2], HIGH);
      }
      else
      {
        digitalWrite(pinSets[i][1], HIGH);
      }
      pinState[i] = motorValue;
    }


    //Serial.print("Got message: "); Serial.print(pinState[i]); Serial.print(" : "); Serial.print(motorValue);
  }
  //Serial.print(";");
}
// Function for sending IMU data 
void sendIMU(imu::Quaternion& quat, imu::Vector<3>& acc)
{
  Serial.print(acc.x(), 4); Serial.print(' '); Serial.print(acc.y(), 4); Serial.print(' '); Serial.print(acc.z(), 4); Serial.print(' ');
  Serial.print(quat.x(), 4); Serial.print(' '); Serial.print(quat.y(), 4); Serial.print(' '); Serial.print(quat.z(), 4); Serial.print(' '); Serial.print(quat.w(), 4);
  Serial.print(';');
}

// Function for waiting for the ROS node to be ready
void confirmCycle()
{
  char readyForCal;
  while (true) {
    if (Serial.available() > 0)
    {
      Serial.readBytes(&readyForCal, 1);
      if (readyForCal == '1') break;
    }
    delay(100);
  }  
}
