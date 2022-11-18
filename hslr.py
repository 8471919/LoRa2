import RPi.GPIO as GPIO
import serial
import time
from util import *

# from sx126x import sx126x

# This Mac Address : e4:5f:01:da:aa:c8
# Opposite Mac Address : e4:5f:01:da:ab:78

# This LoRa's Address is 21
# Opposite LoRa's Address is 100

# To send an image
# This is a protocol of LoRa
class HSLR:
    
    M0 = 22
    M1 = 27
    # if the header is 0xC0, then the LoRa register settings dont lost when it poweroff, and 0xC2 will be lost. 
    # cfg_reg = [0xC0,0x00,0x09,0x00,0x00,0x00,0x62,0x00,0x17,0x00,0x00,0x00]
    cfg_reg = [0xC2,0x00,0x09,0x00,0x00,0x00,0x62,0x00,0x17,0x00,0x00,0x00]
    get_reg = bytes(12)
    rssi = False
    addr = 65535
    serial_n = ""
    send_to = 0
    addr_temp = 0
    freq = 868
    power = 22
    air_speed = 2400

    SX126X_UART_BAUDRATE_1200 = 0x00
    SX126X_UART_BAUDRATE_2400 = 0x20
    SX126X_UART_BAUDRATE_4800 = 0x40
    SX126X_UART_BAUDRATE_9600 = 0x60
    SX126X_UART_BAUDRATE_19200 = 0x80
    SX126X_UART_BAUDRATE_38400 = 0xA0
    SX126X_UART_BAUDRATE_57600 = 0xC0
    SX126X_UART_BAUDRATE_115200 = 0xE0

    SX126X_AIR_SPEED_300bps = 0x00
    SX126X_AIR_SPEED_1200bps = 0x01
    SX126X_AIR_SPEED_2400bps = 0x02
    SX126X_AIR_SPEED_4800bps = 0x03
    SX126X_AIR_SPEED_9600bps = 0x04
    SX126X_AIR_SPEED_19200bps = 0x05
    SX126X_AIR_SPEED_38400bps = 0x06
    SX126X_AIR_SPEED_62500bps = 0x07

    SX126X_PACKAGE_SIZE_240_BYTE = 0x00
    SX126X_PACKAGE_SIZE_128_BYTE = 0x40
    SX126X_PACKAGE_SIZE_64_BYTE = 0x80
    SX126X_PACKAGE_SIZE_32_BYTE = 0xC0

    SX126X_Power_22dBm = 0x00
    SX126X_Power_17dBm = 0x01
    SX126X_Power_13dBm = 0x02
    SX126X_Power_10dBm = 0x03
    
    
    
    def __init__(self, serial_num, freq, addr, power, rssi):
                
        self.HEADER_SIZE = 12
        self.PACKET_SIZE = 240
        self.PAYLOAD_SIZE = 228
        
        # size of Header's components
        self.DEST_EUI_SIZE = 6
        self.SEQUENCE_NUMBER_SIZE = 2
        self.FLAG_SIZE = 1
        self.PAYLOAD_SIZE_OF_SiZE = 1
        self.CHECKSUM_SIZE = 2
        
        # environment setting
        self.FREQUENCY = freq
        self.ADDRESS = addr
        self.POWER = power
        self.RSSI = rssi
        self.SEND_TO = addr
        self.SERIAL_NUMBER = serial_num
    
        # MAC Address
        self.SOURCE_MAC = b'\xe4_\x01\xda\xaa\xc8'  # e4:5f:01:da:aa:c8
        self.DEST_MAC = b'\xe4_\x01\xda\xabx'   # e4:5f:01:da:ab:78
        
        # FLAGS
        self.SYN = 1
        self.ACK = 2
        self.DATA = 3
        self.BVACK = 4
        self.FIN = 5
        self.RBVACK = 6
        
        # To checkt BVACK packet's index
        self.BVACK_INDEX = [1, 2, 3, 4, 5]
        
        # HEADER INDEX
        self.DEST_EUI_INDEX = 0
        self.SEQUENCE_NUMBER_INDEX = 6
        self.FLAG_INDEX = 8
        self.PAYLOAD_SIZE_INDEX = 9
        self.CHECKSUM_INDEX = 10
        
        # Initial the GPIO for M0 and M1 Pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M0,GPIO.OUT)
        GPIO.setup(self.M1,GPIO.OUT)
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)

        # The hardware UART of Pi3B+,Pi4B is /dev/ttyS0
        self.ser = serial.Serial(serial_num,9600)
        self.ser.flushInput()
        self.set(freq,addr,power,rssi)        
        
    def set(self,freq,addr,power,rssi,air_speed=2400,\
            net_id=0,buffer_size = 240,crypt=0,\
            relay=False,lbt=False,wor=False):
        self.SEND_TO = addr
        self.ADDRESS = addr
        
        # We should pull up the M1 pin when sets the module
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)
        time.sleep(0.1)
        low_addr = addr & 0xff
        high_addr = addr >> 8 & 0xff
        net_id_temp = net_id & 0xff
        if freq > 850:
            freq_temp = freq - 850
        elif freq >410:
            freq_temp = freq - 410
        
        air_speed_temp = self.air_speed_cal(air_speed)
        # if air_speed_temp != None:
        
        buffer_size_temp = self.buffer_size_cal(buffer_size)
        # if air_speed_temp != None:
        
        power_temp = self.power_cal(power)
        #if power_temp != None:

        if rssi:
            rssi_temp = 0x80
        else:
            rssi_temp = 0x00

        l_crypt = crypt & 0xff
        h_crypt = crypt >> 8 & 0xff

        self.cfg_reg[3] = high_addr
        self.cfg_reg[4] = low_addr
        self.cfg_reg[5] = net_id_temp
        self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
        # 
        # it will enable to read noise rssi value when add 0x20 as follow
        # 
        self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
        self.cfg_reg[8] = freq_temp
        #
        # it will output a packet rssi value following received message
        # when enable seventh bit with 06H register(rssi_temp = 0x80)
        #
        self.cfg_reg[9] = 0x03 + rssi_temp
        self.cfg_reg[10] = h_crypt
        self.cfg_reg[11] = l_crypt
        self.ser.flushInput()

        for i in range(2):
            self.ser.write(bytes(self.cfg_reg))
            r_buff = 0
            time.sleep(0.2)
            
            # print(self.ser.inWaiting())
            if self.ser.inWaiting() > 0:
                time.sleep(0.1)
                r_buff = self.ser.read(self.ser.inWaiting())
                if r_buff[0] == 0xC1:
                    pass
                    # print("parameters setting is :",end='')
                    # for i in self.cfg_reg:
                        # print(hex(i),end=' ')
                        
                    # print('\r\n')
                    # print("parameters return is  :",end='')
                    # for i in r_buff:
                        # print(hex(i),end=' ')
                    # print('\r\n')
                else:
                    pass
                    #print("parameters setting fail :",r_buff)
                break
            else:
                print("setting fail,setting again")
                self.ser.flushInput()
                time.sleep(0.2)
                print('\x1b[1A',end='\r')
                if i == 1:
                    print("setting fail,Press Esc to Exit and run again")
                    # time.sleep(2)
                    # print('\x1b[1A',end='\r')
                pass

        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.LOW)
        time.sleep(0.1)

    def air_speed_cal(self,airSpeed):
        air_speed_c = {
            1200:self.SX126X_AIR_SPEED_1200bps,
            2400:self.SX126X_AIR_SPEED_2400bps,
            4800:self.SX126X_AIR_SPEED_4800bps,
            9600:self.SX126X_AIR_SPEED_9600bps,
            19200:self.SX126X_AIR_SPEED_19200bps,
            38400:self.SX126X_AIR_SPEED_38400bps,
            62500:self.SX126X_AIR_SPEED_62500bps
        }
        return air_speed_c.get(airSpeed,None)

    def power_cal(self,power):
        power_c = {
            22:self.SX126X_Power_22dBm,
            17:self.SX126X_Power_17dBm,
            13:self.SX126X_Power_13dBm,
            10:self.SX126X_Power_10dBm
        }
        return power_c.get(power,None)

    def buffer_size_cal(self,bufferSize):
        buffer_size_c = {
            240:self.SX126X_PACKAGE_SIZE_240_BYTE,
            128:self.SX126X_PACKAGE_SIZE_128_BYTE,
            64:self.SX126X_PACKAGE_SIZE_64_BYTE,
            32:self.SX126X_PACKAGE_SIZE_32_BYTE
        }
        return buffer_size_c.get(bufferSize,None)

    def get_settings(self):
        # the pin M1 of lora HAT must be high when enter setting mode and get parameters
        GPIO.output(M1,GPIO.HIGH)
        time.sleep(0.1)
        
        # send command to get setting parameters
        self.ser.write(bytes([0xC1,0x00,0x09]))
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            self.get_reg = self.ser.read(self.ser.inWaiting())
        
        # check the return characters from hat and print the setting parameters
        if self.get_reg[0] == 0xC1 and self.get_reg[2] == 0x09:
            fre_temp = self.get_reg[8]
            addr_temp = self.get_reg[3] + self.get_reg[4]
            air_speed_temp = self.get_reg[6] & 0x03
            power_temp = self.get_reg[7] & 0x03
            
            air_speed_dic = {
                0x00:"300bps",
                0x01:"1200bps",
                0x02:"2400bps",
                0x03:"4800bps",
                0x04:"9600bps",
                0x05:"19200bps",
                0x06:"38400bps",
                0x07:"62500bps"
            }
            power_dic ={
                0x00:"22dBm",
                0x01:"17dBm",
                0x02:"13dBm",
                0x03:"10dBm"
            }
            
            print("Frequence is {0}.125MHz.",fre_temp)
            print("Node address is {0}.",addr_temp)
            print("Air speed is "+ air_speed_dic(air_speed_temp))
            print("Power is " + power_dic(power_temp))
            GPIO.output(M1,GPIO.LOW)

    def get_channel_rssi(self):
        GPIO.output(self.M1,GPIO.LOW)
        GPIO.output(self.M0,GPIO.LOW)
        time.sleep(0.1)
        self.ser.flushInput()
        self.ser.write(bytes([0xC0,0xC1,0xC2,0xC3,0x00,0x02]))
        time.sleep(0.5)
        re_temp = bytes(5)
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            re_temp = self.ser.read(self.ser.inWaiting())
        if re_temp[0] == 0xC1 and re_temp[1] == 0x00 and re_temp[2] == 0x02:
            print("the current noise rssi value: -{0}dBm".format(256-re_temp[3]))
            # print("the last receive packet rssi value: -{0}dBm".format(256-re_temp[4]))
        else:
            # pass
            print("receive rssi value fail")
            # print("receive rssi value fail: ",re_temp)
        
    # send Image by LoRa
    def transmitImage(self, imageBytes):
        GPIO.output(self.M1,GPIO.LOW)
        GPIO.output(self.M0,GPIO.LOW)
        time.sleep(0.1)
        
        # turn integer value to byte of 4 byte type using big endian
        imageLengthBytes = len(imageBytes).to_bytes(4, 'big')

        firstPayload = imageLengthBytes + imageBytes[:self.PAYLOAD_SIZE-len(imageLengthBytes)]
    
        packet = self.addHeader(sequenceNum=1, flag=self.SYN, payload=firstPayload)
    
        self.ser.write(packet)
    
        print(firstPayload)
        print(str(len(firstPayload)) + " / " + str(len(len(imageBytes))))
        
        for i in range(self.PAYLOAD_SIZE-len(imageLengthBytes), len(imageBytes), self.PAYLOAD_SIZE):
            time.sleep(2)
            
            packet = self.addHeader(sequenceNum=int(i+len(imageLengthBytes)/self.PAYLOAD_SIZE)+1, flag=self.DATA, payload=imageBytes[i:i+self.PAYLOAD_SIZE])
            self.ser.write(packet)
            print(imageBytes[i:i+self.PAYLOAD_SIZE])
            print(str(i+self.PAYLOAD_SIZE) + " / " + str(len(imageBytes)))


    def receiveImage(self):
        if self.ser.inWaiting() > 0:
            time.sleep(0.5)
            r_buff = self.ser.read(self.ser.inWaiting())
            packet = r_buff[:-1]

            print(packet)
            # check Checksum
            packet = self.check(packet)
            
            payload = bytearray()
            imageBytes = bytearray()
            imageLength = 0
            
            # if packet is not None, get first Packet
            if packet != None:
                payload = packet[self.HEADER_SIZE:]
                # get the image size from first packet
                imageLength = int.from_bytes(payload[:4])
                print("imageLength : " + str(imageLength))
                imageBytes += payload
        
            # get other packets
            while (len(packet) < 240) or (len(imageBytes) < imageLength):
                # time.sleep(0.5)
                if self.ser.inWaiting() > 0:
                    time.sleep(0.5)
                    packet = self.ser.read(self.ser.inWaiting())[:-1]
                    
                    # check Checksum
                    packet = self.check(packet)
                    
                    # if packet is strange, continue
                    if packet == None:
                        print("add BVACK")
                        continue
                    
                    payload = packet[self.HEADER_SIZE:]
                    print(payload)
                    imageBytes += payload
                    
                    print(str(len(imageBytes)) + " / " + str(imageLength))
            
            return imageBytes
                
        
    
    # add header to payload    
    def addHeader(self, sequenceNum, flag, payload):
        if len(payload) > 228:
           print("payload size is over")
           exit()
        
        header = bytearray(12)
        
        # change int to byte
        sequenceNum = sequenceNum.to_bytes(2, 'big')
        flag = flag.to_bytes(1, 'big')
        payloadSize = len(payload).to_bytes(1, 'big')
        
        header[0:6] = self.DEST_MAC
        header[6:8] = sequenceNum
        header[8:9] = flag
        header[9:10] = payloadSize
        
        checkSum = self.calCheckSum(header[:10], payload)
        
        header[10:12] = checkSum
        
        # print("header 0~9 : " + str(header[0:10]))
        # print(len(payload))
        # # calculate checksum
        # # add the each 16bit of the packet
        # sum = 0
        # for i in range(0, 10, 2):
        #     int_val = int.from_bytes(header[i:i+2], 'big')
        #     sum = bin(sum + int_val)[2:]
        #     if len(sum) > 16:
        #         sum = int(sum[1:], 2) + 1
        #     else:
        #         sum = int(sum, 2)
        
        # for i in range(0, len(payload), 2):
        #     int_val = int.from_bytes(payload[i:i+2], 'big')
        #     sum = bin(sum + int_val)[2:]
        #     if len(sum) > 16:
        #         sum = int(sum[1:], 2) + 1
        #     else:
        #         sum = int(sum, 2)
        
        # print("sum : " + str(sum))
        
        # checkSum = sum.to_bytes(2, 'big')
        
        # print("checksum : " + str(checkSum))
                
        # header[10:12] = checkSum
        
        return header + payload
    
    def calCheckSum(self, header, payload):
        tempPacket = header + payload
        
        sum = 0
        for i in range(0, len(tempPacket), 2):
            int_val = int.from_bytes(payload[i:i+2], 'big')
            sum = bin(sum + int_val)[2:]
            if len(sum) > 16:
                sum = int(sum[1:], 2) + 1
            else:
                sum = int(sum, 2)
            
        checkSum = sum.to_bytes(2, 'big')
        
        return checkSum
        
        
    def check(self, packet):

        checkSum = packet[10:12]

        sum = 0
        
        calculatedCheckSum = self.calCheckSum(packet[:10], packet[self.HEADER_SIZE:])
        
        # for i in range(0, 10, 2):
        #     int_val = int.from_bytes(packet[i:i+2], 'big')
        #     sum = bin(sum + int_val)[2:]
        #     if len(sum) > 16:
        #         sum = int(sum[1:], 2) + 1
        #     else:
        #         sum = int(sum, 2)            
        
        # for i in range(12, len(packet), 2):
        #     int_val = int.from_bytes(packet[i:i+2], 'big')
        #     sum = bin(sum + int_val)[2:]
        #     if len(sum) > 16:
        #         sum = int(sum[1:], 2) + 1
        #     else:
        #         sum = int(sum, 2)
                                
        # if checkSum is equal each ohter, return the packet
        # if sum.to_bytes(2, 'big') == bytes(checkSum):
        #     return True
        # else:
        #     return False

        if calculatedCheckSum == bytes(checkSum):
            return True
        else:
            return False


    # check an integrity and remove header of the packet
    def parse(self, packet):
        
        # check DEST EUI
        destEUI = packet[0:6]
        if destEUI != self.DEST_MAC:
            print("destEUI is incorrect")
            return []
        
        # check sequence number
        # 시퀀스 넘버를 BVACK 패킷에 넣는다.
        
        # check flag
        # 플래그 값에 따른 처리
        
        # check payload size
        payloadSize = int.from_bytes(packet[9:10], 'big')
        if len(packet[self.HEADER_SIZE:]) != payloadSize:
            print("length is incorrect")
            return []
                    
        # check CheckSum
        result = self.check(packet)
        if result == False:
            return []
        
        payload = packet[self.HEADER_SIZE:]
        
        return payload

