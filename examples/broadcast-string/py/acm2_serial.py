import re
from rich import print
import serial

ser = serial.Serial(
    port="/dev/ttyACM2",
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
)
print("connected to: " + ser.portstr)
regex = re.compile(r"(?<=\|)(.*?)(?=\|)")

while True:
    line = b""

    while ser.inWaiting() > 0:
        line += ser.readline()

    if line:
        line = line.decode()
        print(line)
        # matchobj = regex.search(line)
        # try:
        #     keyword = matchobj.group().strip()
        #     if keyword == "exit":
        #         break
        #     # print(f"[blue]recv>[/blue] [bold]{keyword}[/bold]")
        # except Exception:
        #     continue

ser.close()
