const uint8_t numberOfPins = 4;
const uint8_t pins[] = {11, 10, 6, 5};
char receivedChars[numberOfPins + 1];   // an array to store the received data

boolean newData = false;

void setup() {
    // Setting pins for output
    for (auto pin : pins){
      pinMode(pin, OUTPUT);
    }
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
}

void loop() {
  if (Serial.available() >= numberOfPins)
  {
    receiveSerial(numberOfPins);
  }
}

void receiveSerial(uint8_t byteNumber)
{
  auto length = Serial.readBytesUntil('\n', receivedChars, byteNumber);

  Serial.print("Received commands of length ");
  Serial.print(length);
  Serial.print(": ");
  Serial.println(receivedChars);

  for (auto i = 0; i < byteNumber; i++)
  {
    digitalWrite(pins[i], receivedChars[i] == '1' ? HIGH : LOW);
  }

}
