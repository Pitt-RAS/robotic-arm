import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portsList = []

for one in ports:
    portsList.append(str(one))
    print(str(one))

serialInst.baudrate = 115200
serialInst.port = "/dev/cu.usbserial-1120"
serialInst.open()

while True:
    command = input() + "\n"
    serialInst.write(command.encode('utf-8'))

    if command == 'exit\n':
        serialInst.close()
        exit()
