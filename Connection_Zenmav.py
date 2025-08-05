from zenmav.core import Zenmav
import time

drone = Zenmav(ip = 'udp:127.0.0.1:14550', baud=115200, GCS = False, tcp_ports=[14551])

#drone = Zenmav('udp:192.168.144.12:19856', baud=115200)