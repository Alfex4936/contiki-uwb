from rich import print
import serial

ser = serial.Serial(
    port="/dev/ttyACM0",
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

    print("[bold magenta]seokwon>[/bold magenta] ", end="")
    text = input()
    ser.write(f"{text}\n".encode("UTF-8"))

    if text == "exit":
        break

ser.close()
