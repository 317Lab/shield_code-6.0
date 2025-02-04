import serial

port = '/dev/cu.usbserial-FT611XTT3'
baudrate = 230400 
output_file = 'output.bin'

ser = serial.Serial(port, baudrate)

try:
    with open(output_file, 'wb') as f:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                f.write(data)
                f.flush()
except KeyboardInterrupt:
    print("Logging stopped by user.")
finally:
    ser.close()