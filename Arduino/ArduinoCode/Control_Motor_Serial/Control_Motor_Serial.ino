const uint8_t numberOfPins = 4;
// 1. PWM Pin, 2. IN1 pin, 3. IN2 pin
int pinSets[][3] = {{11, 13, 12}, {10, 9, 8}, {5,7,6}, {3, 4, 2}};

char receivedChars[numberOfPins * 2];
long int newPinStates[numberOfPins];
long int pinState[numberOfPins];
int deadzone = 5;

void setup() {
  // put your setup code here, to run once:
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
  Serial.println("Arduino Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() >= numberOfPins * 2)
  {
    receiveSerial(numberOfPins * 2);
    for (auto &&num: receivedChars)
    {
      Serial.print(num); Serial.print(" , ");
    }
    Serial.println();
    controlMotor(newPinStates);
  }

}

void receiveSerial(int byteNumber)
{
  auto length = Serial.readBytesUntil('\n', receivedChars, byteNumber);

  for (auto i = 0; i < byteNumber/2; i++)
  {
    int newNum = receivedChars[i*2+1];
    if (receivedChars[i*2] == '-')
    {
      newNum = newNum * -1;
    }
    newPinStates[i] = newNum;
  }
}

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


    Serial.print("Got message: "); Serial.print(pinState[i]); Serial.print(" : "); Serial.println(motorValue);
  }

}
