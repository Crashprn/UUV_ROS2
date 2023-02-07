#!/usr/bin/env python3
import serial
import os
import time
import keyboard


class SerialDigital:
    def __init__(self, serial:serial.Serial):
        self.pins = [0,0,0,0]
        self.serial = serial
    
    def writeMsg(self, pinValues, serial: serial.Serial):
        msg = ''
        for pin in pinValues:
            if pin:
                msg += '1'
            else:
                msg += '0'
                 
        serial.write(msg.encode())
        
    @staticmethod
    def flipBool(pin):
        return 0 if pin else 1   

    def aCallback(self):
        self.pins[0] = self.flipBool(self.pins[0])
        self.writeMsg(self.pins, self.serial)
        time.sleep(0.001)
        print(self.serial.read_all())

    def sCallback(self):
        self.pins[1] = self.flipBool(self.pins[1])
        self.writeMsg(self.pins, self.serial)
        time.sleep(0.001)
        print(self.serial.read_all())

    def dCallback(self):
        self.pins[2] = self.flipBool(self.pins[2])
        self.writeMsg(self.pins, self.serial)     
        time.sleep(0.001)
        print(self.serial.read_all())

    def fCallback(self):
        self.pins[3] = self.flipBool(self.pins[3])
        self.writeMsg(self.pins, self.serial)
        time.sleep(0.001)
        print(self.serial.read_all())

    
def main():
    SerialPortName = os.path.join("/dev", "ttyACM0")

    SerialPortObj = serial.Serial(SerialPortName)

    SerialPortObj.baudrate = 9600
    SerialPortObj.timeout = 0

    serialObj = SerialDigital(SerialPortObj)

    print(f"Opened Serial Port{SerialPortObj}")

    time.sleep(5)

    print("Ready for input")
    
    keyboard.add_hotkey('a', serialObj.aCallback)
    keyboard.add_hotkey('s', serialObj.sCallback)
    keyboard.add_hotkey('d', serialObj.dCallback)
    keyboard.add_hotkey('f', serialObj.fCallback)
    
    while(True):
        time.sleep(0.001)
        pass
        
if __name__ == "__main__":
    main()
