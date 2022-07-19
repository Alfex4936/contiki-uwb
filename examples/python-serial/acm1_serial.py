import serial

ser = serial.Serial(
    port="/dev/ttyACM1",
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
)
print("connected to: " + ser.portstr)

while True:
    line = b""

    while ser.inWaiting() > 0:
        line += ser.readline()

    if line:
        print(line.decode())

ser.close()
