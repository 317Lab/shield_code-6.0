# script for uploading firmware with bossa without rebuilding the project
# TODO: check if you can exclude port specification, include windows and linux support
import os
import subprocess
import re
import time
filepath = ".pio/build/due/firmware.bin"
# bossa port format doesn't match above ls so we have to reconstruct
mac_port = subprocess.check_output("ls /dev/tty.usb*", shell=True, text=True).strip()
port_num = re.findall(r'\d+', mac_port)[0]
port = "/dev/cu.usbmodem"+str(port_num)
# send 1200 bps signal to enter boot loading mode
subprocess.run(["stty","-f",port,"1200"],check=True)
# double check port
mac_port = subprocess.check_output("ls /dev/tty.usb*", shell=True, text=True).strip()
port_num = re.findall(r'\d+', mac_port)[0]
# name format different for bossa
port = "cu.usbmodem"+str(port_num)
# wait for boot loader
time.sleep(2)
# upload firmware with bossac.
# -U must be true for native port upload (bobshieldv6)
command = "~/Library/Arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac --port="+port+" -U true -e -w -v -b "+filepath+" -R"
os.system(command)
