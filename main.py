from hslr import HSLR

def main():
    lora = HSLR(serial_num='/dev/ttyS0', freq=915, addr=21, power=22, rssi=True)
    
    a = b'\xe4_\x01\xda\xabx'

    packet = lora.addHeader(sequenceNum=1, flag=1, payload=a)
    
    print(packet)
    
    print("parse")
    
    lora.check(packet)
    
    

if __name__=="__main__":
    main()