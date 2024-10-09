#
#  Update settings on an m5 gps
#
from serial import Serial

PORT="COM8" # Pearl
PORT="/dev/ttyUSB0" # Murre
BPS = 9600 # factory setting
#BPS = 115200 # after reprogramming


def checksum(sentence):
    # Given the body of a NMEA sentence calculate its checksum.
    calc_cksum = 0
    for s in sentence:
        calc_cksum ^= ord(s)
    return "%0.2X" % calc_cksum


def buildnmea(talker, cmd, payload):
    msg = talker + cmd + ',' + payload
    cs = checksum(msg)
    cmd = '$' + msg + '*' + cs + '\r\n'
    return cmd.encode('utf-8')


stream = Serial(PORT, BPS, timeout=3)

# Best is probably GPS and Beidou
msg = buildnmea('P', 'CAS04', '3') # GPS, BDS
#msg = buildnmea('P', 'CAS04', '5') # GPS, GLONASS
#msg = buildnmea('P', 'CAS04', '7') # BDS, GPS, GLONASS
print(msg)
stream.write(msg)

# Change sentences returned
# 0 means never
# 1 means every cycle
# 4 means every 4th cycle
# '' don't change 
flags = [
'5', #GGA 2   position, hdop
'0', #GLL 3
'5', #GSA 4   sats available
'5', #GSV 5   sats in use
'1', #RMC 6   time,ll,speed,heading,alt
'0', #VTG 7   
'0', #ZDA 8
'0', #ANT 9
'0', #DHV 10
'0', #LPS 11
'', #res1 12
'', #res2 13
'0', #UTC 14
'0', #GST 15
'', #res3 16
'', #res4 17
'', #res5 18
'0', #TIM 19
]
payload = ','.join(flags)
msg = buildnmea('P', 'CAS03', payload)
print(msg)
stream.write(msg)
                
# Change data rate
#msg = buildnmea('P', 'CAS01', '1') # 9600
msg = buildnmea('P', 'CAS01', '5') # 115200
print(msg)
stream.write(msg)

# Change update rate
#msg = buildnmea('P', 'CAS02', '1000') # 2/second
#msg = buildnmea('P', 'CAS02', '500') # 2/second
msg = buildnmea('P', 'CAS02', '200') # 5/second
print(msg)
stream.write(msg)
