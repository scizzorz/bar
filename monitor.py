import serial

port = serial.Serial("/dev/ttyACM0", 9600)

while True:
    print(port.readline().decode("utf-8"), end='')
