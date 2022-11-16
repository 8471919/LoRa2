#!/usr/bin/python
# -*- coding: UTF-8 -*-

#
#    this is an UART-LoRa device and thers is an firmware on Module
#    users can transfer or receive the data directly by UART and dont
#    need to set parameters like coderate,spread factor,etc.
#    |============================================ |
#    |   It does not suport LoRaWAN protocol !!!   |
#    | ============================================|
#   
#    This script is mainly for Raspberry Pi 3B+, 4B, and Zero series
#    Since PC/Laptop does not have GPIO to control HAT, it should be configured by
#    GUI and while setting the jumpers, 
#    Please refer to another script pc_main.py
#

import sys
from . import sx126x
import threading
import time
import select
import termios
import tty
from threading import Timer

import json

class LoRa:
    """property

    """

    def __init__(self):
        
        self.serial_num = "/dev/ttyS0"
        self.freq = 915
        # self.freq = 433
        # send to who
        self.addr = 21
        self.power = 22
        self.rssi = True
        
        # it will send every seconds(default is 10) seconds 
        # send_to_who is the address of other node ( defult is 21)
        self.send_to_who = 100
        self.seconds = 5
        
        self.node = sx126x.sx126x(serial_num = self.serial_num, freq=self.freq, addr=self.addr, power=self.power, rssi=self.rssi)

    def sendType(self, typeDic):
        
        # node setting
        self.node.addr_temp = self.node.addr
        self.node.set(self.node.freq, self.send_to_who, self.node.power, self.node.rssi)
        
        payload = json.dumps(typeDic)
        print("payload")
        print(payload)
        
        # send the payload
        self.node.transmitType(payload)

        self.node.set(self.node.freq, self.node.addr_temp, self.node.power, self.node.rssi)
        
    # get only { sound : 1 } or { start : 1 } not img
    def getPacket(self):
        
        processed = self.node.receive()

        if processed != None:
            result = json.loads(processed)
            
            return result
        
        return {}

    def getImage(self):
        
        while True:
            imageBytes = self.node.receiveImage()
            
            if imageBytes != None:
                break
            
        
        return imageBytes

