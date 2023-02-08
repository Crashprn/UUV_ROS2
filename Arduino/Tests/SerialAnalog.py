from serial import Serial
import os
import sys
import time


class SerialAnalog:
    
    def __init__(self, ser: Serial):
        self.serial = ser
        print(ser)
    
    def writeMsg(self, pins):
        msg = ''
        for pin in pins:
            msg += pin
        self.serial.write(msg.encode())



def main(args = None):
    portPath = os.path.join('/dev', 'ttyACM0')
    serialObj = SerialAnalog(Serial(portPath))
    
    time.sleep(4)
    
    while True:
        pinValues = input("Enter space seperated values from 0 to 255: ")
        inputArray = pinValues.split()
        val_ints = []
        for value in inputArray:
            val_int = value
            val_ints.append(val_int)
        
        serialObj.writeMsg(val_ints)
        time.sleep(0.001)                       

def main2():
    portPath = os.path.join('/dev', 'ttyACM0')
    serialObj = SerialAnalog(Serial(portPath))
    
    time.sleep(4)
    
    stepSize = 5
    pins = 4
    while True:
        for i in range(255 //stepSize):
            serialObj.writeMsg([i * stepSize for j in range(pins)])
            time.sleep(0.001)
        for i in range(255//stepSize, 0, -1):
            serialObj.writeMsg([i * stepSize for j in range(pins)])
            time.sleep(0.001)
        
        
if __name__ == "__main__":
    if sys.argv[1] == '0':
        main()
    else:
        main2()