import serial

ser = serial.Serial(
    port="/dev/ttyACM0",
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0,
)
print("connected to: " + ser.portstr)

while True:
    text = input("Enter: ")
    ser.write(f"{text}\n".encode("UTF-8"))

    if text == "exit":
        break

ser.close()
