import serial
import struct
from datetime import datetime
import time
from netifaces import interfaces, ifaddresses, AF_INET

#TTY="/dev/ttyACM0" # Asl
TTY="/dev/ttyUSB0" # Murre
EOF = struct.pack('B', 0xFF)

debug = False
elapsed = 0

def getIPaddress() -> str:
    ip = "No network"
    for ifaceName in interfaces():
        addies = ifaddresses(ifaceName)
        #print(addies)
        addresses = [ i["addr"] for i in addies.setdefault(AF_INET, [{"addr": "0"}])]
        #print(ifaceName," ".join(addresses))
        if ifaceName == 'wlan0' and addresses[0] != 0:
        # prefer the wlan address
            ip = "wlan0 " + addresses[0]
            try:
              # try to read the current SSID
              ssid = os.popen('iwgetid -r').read().strip()
            except Exception as e:
              ssid = '???'
            #print(ssid)
            ip += ' ' + ssid
        elif ifaceName == 'eth0' and addresses[0] != 0:
            ip = "eth0 " + addresses[0]

    return ip


port=serial.Serial(
    port=TTY,
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS, 
    timeout=1)

def sendCmd(s: str) -> None:
    if debug: print(s)
    port.write(str.encode(s))
    port.write(EOF)
    port.write(EOF)
    port.write(EOF)
    return

def setBrightness(level:int) -> None:
    dimCmd = f"dim={level:d}"
    sendCmd(dimCmd)
    return

setBrightness(100) # Turn on screen 100%
exit(0)

sendCmd("page 0") # Set to page 0
sendCmd("vis gps,0") # Turn off satellite picture

while True: 
    x = datetime.now()
    timestamp = x.strftime("%I:%M:%S %m/%d/%y")
    tscmd = f'page0.datetime.txt="{timestamp}"'
    sendCmd(tscmd)

    # I think I could make this a scrolling textbox and put more than one address here
    ip = getIPaddress()
    sendCmd(f'page0.network.txt="{ip}"');

    sendCmd(f'page0.log.txt="{elapsed}\n"')

    time.sleep(1)
    elapsed += 1
