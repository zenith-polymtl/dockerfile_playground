from zenmav.core import Zenmav
import time

drone = Zenmav(ip = 'udp:127.0.0.1:14551')


#drone = Zenmav(ip = '/dev/serial0', baud=115200)

#drone = Zenmav('udp:192.168.144.12:19856', baud=115200)