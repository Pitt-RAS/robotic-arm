import serial
import time

# The port variable might have to be changed depending on what computer it's being run on
# To check what port the Arduino is on, go into Arduino IDE, click "tools" in the top-of-screen menu, and you should see it there
arduino = serial.Serial(port='/dev/cu.usbmodem111201', baudrate=115200, timeout=.1)

def send_to_arduino(angle_to_send):
    arduino.write(bytes(angle_to_send, 'utf-8'))

while True:
    angle = input("Enter a number between 0 and 180: ")
    # Maybe add a check to make sure it's not a negative number
    value = send_to_arduino(angle)
