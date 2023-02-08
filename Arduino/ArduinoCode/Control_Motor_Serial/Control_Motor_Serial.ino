const uint8_t numberOfPins = 1;
char receivedChars[numberOfPins * 2];
long int pinState[numberOfPins];
int x_in = A0;
long int x_pos;
int deadzone = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
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
    controlMotor(receiveSerial(numberOfPins * 2));
  }

}

long int receiveSerial(int byteNumber)
{
  auto length = Serial.readBytesUntil('\n', receivedChars, byteNumber);

  int newNum = static_cast<uint8_t>(receivedChars[1]);

  if (receivedChars[0] == '-')
  {
    newNum = newNum * -1;
  }
  return newNum;
}

void controlMotor(long int motorValue)
{

  if (abs(motorValue) < deadzone || (motorValue < 0 && pinState[0] > 0) || (motorValue > 0 && pinState[0] < 0))
  {
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    pinState[0] = 0;
    delay(200);
  }
  else 
  {
    analogWrite(11, abs(motorValue));

    if (motorValue < 0)
    {
      digitalWrite(7, HIGH);
    }
    else
    {
      digitalWrite(8, HIGH);
    }
    pinState[0] = motorValue;
  }

  Serial.print("Got message: "); Serial.print(pinState[0]); Serial.print(" : "); Serial.println(motorValue);
}
