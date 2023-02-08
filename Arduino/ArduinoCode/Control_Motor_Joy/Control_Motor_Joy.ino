const uint8_t numberOfPins = 1;
char receivedChars[numberOfPins * 2];
long int pinState[numberOfPins];
int x_in = A0;
long int x_pos;
int deadzone = 8;

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
}

void loop() {
  // put your main code here, to run repeatedly:
  x_pos = analogRead(x_in) - 511;
  
  controlMotor(x_pos / 2);

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
}
