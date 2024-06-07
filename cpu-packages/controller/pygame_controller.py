import serial #for Serial communication
import time   #for delay functions
import math
import serial
from struct import *
import sys
import time
import random
import ast
import pygame

# Define serial connection
try:
    arduino1 = serial.Serial(baudrate=9600, timeout=0.5, port="/dev/ttyUSB0")
    # arduino2 = serial.Serial(baudrate=9600, timeout=0.5, port="/dev/ttyACM0")
except Exception as e:
    print('Port open error', e)

arduino1.flushInput()
# arduino2.flushInput()
time.sleep(5)
print(arduino1.readline())
# print(arduino2.readline())

speed_pwm = 240

# Function to send data to Arduino
def send_data(stop=False, forward=False, backward=False, left=False, right=False):
    if stop:
        print("Stopping...")
        arduino1.write(pack('9h', 0, 0, 0, 0, 0, 0, 0, 0, -1))
        # print(arduino1.readline())
    elif forward:
        print("Moving Forward...")
        arduino1.write(pack('9h', 0, speed_pwm, 0, speed_pwm, 0, speed_pwm, 0, speed_pwm, 100))
        # print(arduino1.readline())
    elif backward:
        print("Moving Backward...")
        arduino1.write(pack('9h', speed_pwm, 0, speed_pwm, 0, speed_pwm, 0, speed_pwm, 0, 100))
        # print(arduino1.readline())
    elif right:
        print("Moving Right...")
        arduino1.write(pack('9h', speed_pwm, 0, 0, speed_pwm, 0, speed_pwm, speed_pwm, 0, 100))
        # arduino1.write(pack('9h', speed_pwm, 0, speed_pwm, 0, 0, speed_pwm, 0, speed_pwm, 100))
        # print(arduino1.readline())
    elif left:
        print("Moving Left...")
        arduino1.write(pack('9h', 0, speed_pwm, speed_pwm, 0, speed_pwm, 0, 0, speed_pwm, 100))
        # print(arduino1.readline())

# Function to handle user input and send motor control values
def handle_input():
    running = True

    while running:
        # print(arduino1.readline())
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    send_data(forward=True)
                elif event.key == pygame.K_DOWN:
                    send_data(backward=True)
                elif event.key == pygame.K_LEFT:
                    send_data(left=True)
                elif event.key == pygame.K_RIGHT:
                    send_data(right=True)
                elif event.key == pygame.K_SPACE:
                    send_data(stop=True)
                    running = False

if __name__ == "__main__":
    pygame.init()
    pygame.display.set_caption("Robot Control")

    screen = pygame.display.set_mode((200, 200))
    clock = pygame.time.Clock()

    while True:
        try:
            handle_input()
        except KeyboardInterrupt:
            break
        except:
            print(str(sys.exc_info()))
            break

    pygame.quit()
