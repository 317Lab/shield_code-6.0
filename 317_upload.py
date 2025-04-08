# script for uploading firmware with bossa without rebuilding the project
# TODO: check if you can exclude port specification, include windows and linux support
import os
import subprocess
import re
filepath = ".pio/build/due/firmware.bin"
# bossa port format doesn't match above ls so we have to reconstruct
mac_port = subprocess.check_output("ls /dev/tty.usb*", shell=True, text=True).strip()
print(mac_port)
port_num = re.findall(r'\d+', mac_port)[0]
print(port_num)
port = "cu.usbmodem"+str(port_num)
# send 1200 bps signal to enter boot loading mode
command = "stty 1200 < /dev/"+port
os.system(command)
# upload firmware with bossac.
# -U must be true for native port upload (bobshieldv6)
command = "~/Library/Arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac -i -d --port="+port+" -U true -e -w -v -b "+filepath
os.system(command)
