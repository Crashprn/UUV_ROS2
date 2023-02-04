const uint8_t numberOfPins = 4;
const uint8_t pins[] = {11, 10, 6, 5};
char receivedChars[numberOfPins+1];

void setup() {
  // put your setup code here, to run once:
  for(auto pin : pins)
  {
    pinMode(pin, OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() >= numberOfPins)
  {
    receiveSerial(numberOfPins);
  }
}

void receiveSerial(uint8_t byteNumber)
{
  auto length = Serial.readBytesUntil('\n', receivedChars, byteNumber);

  for (auto i = 0; i < byteNumber; i++)
  {
    analogWrite(pins[i], static_cast<int>(receivedChars[i]));
  }
}