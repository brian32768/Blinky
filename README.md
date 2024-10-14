# Blinky

Started as a program to blink the led on a Pi Pico.
Now supports GPS on a UART port.
Parses NMEA using minmea and spits it out
on the USB UART port via the Pi Debug Probe.

I have a Raspberry Pi 5 running openocd, like this:

```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "bindto 0.0.0.
0"
```

A Pi Debug Module plugs into USB on the Pi 5. I can watch the serial ports using Minicom on the Pi 5.
Listen to the console,

```bash
minicom -D /dev/ttyACM1
```

I can write code on the desktop Murre or laptop Pearl
using Visual Studio Code with the Pi Pico profile installed.
The debugger works, this is so cool.

## Serial ports on Pico

With the USB cable plugged in, on the Pi 5, I can see

```bash
crw-rw---- 1 root plugdev 166,  1 Oct  6 18:57 /dev/ttyACM1
crw-rw---- 1 root dialout 204, 64 Oct  3 21:39 /dev/ttyAMA0
crw-rw---- 1 root dialout 204, 74 Oct  3 21:40 /dev/ttyAMA10
```

I can connect minicom and see the console output on ttyACM1.

When I plug in the Debug Probe, I can see it show up in dmesg,

```bash
[249442.079651] usb 3-2: new full-speed USB device number 110 using xhci-hcd
[249447.445255] usb 3-2: New USB device found, idVendor=2e8a, idProduct=000a, bcdDevice= 1.00
[249447.445264] usb 3-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[249447.445268] usb 3-2: Product: Pico
[249447.445270] usb 3-2: Manufacturer: Raspberry Pi
[249447.445273] usb 3-2: SerialNumber: E660C062134C4F29
[249447.480317] cdc_acm 3-2:1.0: ttyACM1: USB ACM device
[249557.809819] pl011-axi 1f00030000.serial: DMA channel TX dma2chan2
[249557.809834] pl011-axi 1f00030000.serial: DMA channel RX dma2chan3
[322310.079637] usb 1-1: USB disconnect, device number 2
[322489.218580] usb 1-1: new full-speed USB device number 3 using xhci-hcd
[322489.396527] usb 1-1: New USB device found, idVendor=2e8a, idProduct=000c, bcdDevice= 1.01
[322489.396533] usb 1-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[322489.396537] usb 1-1: Product: Debug Probe (CMSIS-DAP)
[322489.396539] usb 1-1: Manufacturer: Raspberry Pi
[322489.396542] usb 1-1: SerialNumber: E6632891E36B9730
[322489.426598] cdc_acm 1-1:1.1: ttyACM0: USB ACM device
```

The tty ports now show up as

```bash
crw-rw---- 1 root plugdev 166,  0 Oct  7 15:14 /dev/ttyACM0
crw-rw---- 1 root plugdev 166,  1 Oct  6 18:57 /dev/ttyACM1
crw-rw---- 1 root dialout 204, 64 Oct  3 21:39 /dev/ttyAMA0
crw-rw---- 1 root dialout 204, 74 Oct  3 21:40 /dev/ttyAMA10
```

This leads me to conclude that the Debug Probe serial port is ttyACM0.
Connecting via minicom with "minicom -D /dev/ttyACM0 -b 9600" shows nothing. Perhaps I got the wires backwards. Yup. They should be:

### Debug Module UART to Pico UART1

* Yellow wire (RX) to pin 11 aka GP8 aka TX1
* Orange wire (TX) to pin 12 aka GP9 aka RX1
* Black wire to pin 13 aka GND

### Bus Pirate 3

See https://learn.sparkfun.com/tutorials/bus-pirate-v36a-hookup-guide

Brown = GND
Red = 3V3
Orange = 5V
Yellow = ADC
Green = VPU
Blue = AUX
Violet = CLK
Grey = MOSI (TX)
White = CS
Black = MISO (RX)

### Pico UART1 to Nextion NX3224TO24

* Yellow wire (RX) to pin 11 aka GP8 aka TX1
* Blue wire (TX) to pin 12 aka GP9 aka RX1
* Red to +5
* Black to GND

### Nextion tester

Connection Nextion directly to host using USB-UART cable
or UART port on Debug Module.

```bash
conda create --name=nextion python
conda activate nextion
conda install pyserial netifaces
python nextion.py
```

This works, when the python script runs, you can see the
time changing on the Nextion. Tick tock.

So, I have a program that can send command to a Nextion using Python
from either a Linux desktop (Murre) or a Pi 5 (ASL).
I need it to work from the Pico. 

#### Next steps

1. Connect the Bus Pirate to the USB-UART adapter and see what the
strings look like coming from Murre. There should be no surprises.

* Set both sides to 9600 8N1. (Use Bus Pirate interactive mode)
* Bus pirate Brown to UART Black
* Bus pirate Black (RX) to UART Green
* Bus pirate Brown (TX) to UART White
In Bus Pirate, 
* Use 'm' to set up UART mode (including setting to 9600).
* Use '{' to set it to receive bytes from UART.
Data received will display in the Bus Pirate in hex mode. 

Run the nextion.py program and observe. There should be a command
each second, followed by three 0xFF characters.

This is what it looks like when it captures a screen bright command.

```bash
READ: 0x69    d
READ: 0x69    i                                      
READ: 0x6D    m                          
READ: 0x3D    =                          
READ: 0x31    1                            
READ: 0x30    0                   
READ: 0x30    0                
READ: 0xFF                                 
READ: 0xFF                                  
READ: 0xFF          
```

2. Connect the Bus Pirate to the Pico. The strings should look the same.
If not, figure out why not.

AH. Well I had EOF set to 0x1F instead of 0xFF. That was it.

## GPS

The GPS receiver is the M5Stack BDS/GPS Grove module. Based on the AT6558 chip by Casic. I need to reprogram the GPS, as received it does not
put out the sentences I want.

### GPS Grove connection to Pico

* White wire (RX) to pin 1 (GP0) TX0
* Yellow wire (TX) to pin 2 (GP1) RX0
* Red wire pin 40 VBUS
* Black wire GND

This chip can receive any combination of Beidou, GLONASS
and Navstar.

Supported sentences by default include

GSA sats available and fix and DOP
GSV sats in view, elevation, azimuth
GLL lat lon and time
TXT model # and antenna is "open", it's using an internal antenna

* There is a button cell to facilitate hot restarts
* The LED is wired directly to its 1PPS output.

## Reprogramming BDS/GPS modules

Query product information

$PCAS06,0*<cs>\r\n # firmware version
$PCAS06,1*<cs>\r\n # hardware and serial #

Change baud rate from 9600 to 115200 (to accommodate update rate change)

$PCAS01,5*<cs>\r\n

Change update rate (just for the heck of it)

$PCAS02,1000*<cs>\r\n set to 1000 ms

$PCAS02,100*<cs>\r\n set to 100 ms

Change from Beidou/GLONASS to GPS/GLONASS

$PCAS04,5*<cs>\r\n

Add RMC sentence
Remove GLL (redundant with RMC turned on)
Remove ANT message

$PCAS03,0,,,1,,0,10,,,,,,,,,,*<cs>\r\n

Compatible with NMEA 4.1

$PCAS05,2*<cs>\r\n

I wrote a Python program to change settings.
I used a USB to TTL adapter and connected the 
GPS directly to my Windows laptop.

The program is called **m5gps_settings.py**.

## Set up Python environment

conda create --name=gps
conda activate gps
conda install -c conda-forge pynmeagps pyserial

Once you have reprogrammed a unit, it will
need further commands sent at 115200 instead of 9600.
That's set directly in the code so you have to change it.

### Some test notes

I have 3 modules. On power up, each unit reports the same firmware but a unique serial number, that's good.

#3 seems weaker than the other two. Maybe it will be okay outdoors. I feel it might need a new battery too, it seems
to need a full cold start every time I power it up.

#1 says

$GPTXT,01,01,02,MA=CASIC*27
$GPTXT,01,01,02,HW=ATGM336H,0001170603610*17
$GPTXT,01,01,02,IC=AT6558-5N-31-0C510800,BM05CKJ-F2-015708*51
$GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
$GPTXT,01,01,02,TB=2020-04-28,13:43:10*40
$GPTXT,01,01,02,MO=GB*77
$GPTXT,01,01,02,BS=SOC_BootLoader,V6.2.0.2*34
$GPTXT,01,01,02,FI=00856014*71

#2 says

$GPTXT,01,01,02,MA=CASIC*27
$GPTXT,01,01,02,HW=ATGM336H,0001230402589*10
$GPTXT,01,01,02,IC=AT6558-5N-31-0C510800,BMH9CKJ-D2-011398*2E
$GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
$GPTXT,01,01,02,TB=2020-04-28,13:43:10*40
$GPTXT,01,01,02,MO=GB*77
$GPTXT,01,01,02,BS=SOC_BootLoader,V6.2.0.2*34
$GPTXT,01,01,02,FI=00856014*71

#3 says

$GPTXT,01,01,02,MA=CASIC*27
$GPTXT,01,01,02,HW=ATGM336H,0001170605353*13
$GPTXT,01,01,02,IC=AT6558-5N-31-0C510800,BH17CKJ-D1-020079*51
$GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
$GPTXT,01,01,02,TB=2020-04-28,13:43:10*40
$GPTXT,01,01,02,MO=GB*77
$GPTXT,01,01,02,BS=SOC_BootLoader,V6.2.0.2*34
$GPTXT,01,01,02,FI=00856014*71

## Design the HMI in the Nextion Editor

It's a Windows dotnet35 program. I have it installed on Pearl under Windows 11 but also got it running in WINE.

I followed instructions I found here:
https://forum.winehq.org/viewtopic.php?p=128808#p128808

```bash
apt install wine winetricks
sudo winetricks --self-update
WINEPREFIX=~/.nextion wine nextion-setup-v1-67-1.exe
```

I had to resize the fonts to make them visible, this works fine in 
WINE, probably some way to change it in Windows too if I care to try.
https://forum.winehq.org/viewtopic.php?t=27330

```bash
WINEPREFIX=~/.nextion winecfg
```

### Running Nextion Editor in WINE

There is a symlink so the Nextion files in WINE will show up in 
C:/Users/bwilson/Documents/Dropbox/Projects/RP2040/Blinky/Nextion/

```bash
cd ~/.nextion/drive_c/
cp alba.* ~/.nextion/drive_c/
```
Running it is like this

```bash
cd ~/.nextion/drive_c/Program\ Files/Nextion\ Editor/
WINEPREFIX=~/.nextion wine Nextion\ Editor.exe
```

## Resources

M5stack GPS/BDS unit based on Casic AT6558

https://espruino.microcosm.app/api/v1/files/68b597874e0a617692ebebc6a878032f76b34272.pdf

