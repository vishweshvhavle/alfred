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
import rospy
from visualization_msgs.msg import MarkerArray

class Controller:

    MAX_PWM = 255
    MIN_PWM = 180
    WHEEL_DISTANCE = 0.5 # in meters

    ANGULAR_K = 5
    LINEAR_K = 1

    def __init__(self) -> None:
        self.arduino = self.initialize_arduino()
        rospy.init_node("gym", anonymous=True)
        self.angular_subscriber = rospy.Subscriber("angular_velocity", MarkerArray, self.angular_callback, queue_size=1)
        self.linear_subscriber = rospy.Subscriber("linear_velocity", MarkerArray, self.linear_callback, queue_size=1)
        self.linear = 0
        self.angular = 0
        self.running = False
        self.auto_mode = False

        self.linear_constant = self.LINEAR_K
        self.angular_constant = self.ANGULAR_K

    def linear_callback(self, data: MarkerArray):
        self.linear = data.markers[0].scale.x
        self.linear *= self.linear_constant
        # print("Changing linear velocity to: ", self.linear)

    def angular_callback(self, data: MarkerArray):
        self.angular = data.markers[0].scale.x
        self.angular *= self.angular_constant
        # print("Changing angular velocity to: ", self.angular)

    def initialize_arduino(self):
        try:
            arduino1 = serial.Serial(baudrate=9600, timeout=0.5, port="/dev/ttyUSB0")
        except Exception as e:
            print('Port open error', e)

        arduino1.flushInput()
        time.sleep(5)
        print(arduino1.readline())
        return arduino1
    
    def send_data(self, right_RPWM, right_LPWM, left_RPWM, left_LPWM):
        print("right_RPWM:", right_RPWM, "right_LPWM:", right_LPWM, "\nleft_RPWM:", left_RPWM, "left_LPWM:", left_LPWM)
        top_left_wheel = (left_LPWM, left_RPWM)
        bottom_left_wheel = (left_LPWM, left_RPWM)
        top_right_wheel = (right_LPWM, right_RPWM)
        bottom_right_wheel = (right_LPWM, right_RPWM)

        # print(top_left_wheel, bottom_right_wheel, top_right_wheel, bottom_left_wheel)

        self.arduino.write(
            pack(
                '9h', 
                bottom_right_wheel[0], 
                bottom_right_wheel[1], 
                bottom_left_wheel[0], 
                bottom_left_wheel[1], 
                top_left_wheel[0], 
                top_left_wheel[1], 
                top_right_wheel[0], 
                top_right_wheel[1], 
                100
            )
        )
        return

    def map_RPWM(self, vel):
        RPWM = 0
        if vel > 0:
            RPWM = (self.MAX_PWM - self.MIN_PWM)/(self.linear_constant*1 + self.angular_constant*self.WHEEL_DISTANCE/2) * vel + self.MIN_PWM
        
        return int(RPWM)
    
    def map_LPWM(self, vel):
        LPWM = 0
        if vel < 0:
            LPWM = -(self.MAX_PWM - self.MIN_PWM)/(self.linear_constant*1 + self.angular_constant*self.WHEEL_DISTANCE/2) * vel + self.MIN_PWM
        
        return int(LPWM)

    def create_pwm(self):
        print("\nlinear velocity:", self.linear, "\nangular velocity:", self.angular)
        left_vel= self.linear + self.angular * self.WHEEL_DISTANCE/2
        right_vel = self.linear - self.angular * self.WHEEL_DISTANCE/2
        print("left_vel:", left_vel, "\nright_vel:", right_vel)

        right_RPWM = self.map_RPWM(right_vel)
        right_LPWM = self.map_LPWM(right_vel)

        left_RPWM = self.map_RPWM(left_vel)
        left_LPWM = self.map_LPWM(left_vel)

        print("left_LPWM:", left_LPWM, "left_RPWM:", left_RPWM, "\nright_LPWM:", right_LPWM, "right_RPWM:", right_RPWM)
        return right_RPWM, right_LPWM, left_RPWM, left_LPWM

    def run_fluct(self):
        # change the RPWM value from 0 to 255 and back to 0 then change lpwm value from 0 to 255 and back to 0 and repeat
        right_RPWM, right_LPWM, left_RPWM, left_LPWM = 0, 0, 0, 0
        try:
            while True:
                for i in range(150, 255):
                    right_RPWM = i
                    left_RPWM = i
                    self.send_data(right_RPWM, right_LPWM, left_RPWM, left_LPWM)
                    time.sleep(0.1)
                for i in range(255, 150, -1):
                    right_RPWM = i
                    left_RPWM = i
                    self.send_data(right_RPWM, right_LPWM, left_RPWM, left_LPWM)
                    time.sleep(0.1)
                for i in range(150, 255):
                    right_LPWM = i
                    left_LPWM = i
                    self.send_data(right_RPWM, right_LPWM, left_RPWM, left_LPWM)
                    time.sleep(0.1)
                for i in range(255, 150, -1):
                    right_LPWM = i
                    left_LPWM = i
                    self.send_data(right_RPWM, right_LPWM, left_RPWM, left_LPWM)
                    time.sleep(0.1)

        except:
            self.send_data(0, 0, 0, 0)
            sys.exit()
    

    def run(self):
        right_RPWM, right_LPWM, left_RPWM, left_LPWM = 0, 0, 0, 0

        while True:

            if self.auto_mode:
                right_RPWM, right_LPWM, left_RPWM, left_LPWM = self.create_pwm()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    right_RPWM, right_LPWM, left_RPWM, left_LPWM = 0, 0, 0, 0  
                    self.send_data(0, 0, 0, 0)
                    self.running = False
                    return 
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_s:
                        self.running = True
                        print("Starting the robot...")

                    elif event.key == pygame.K_a:
                        self.auto_mode = True if not self.auto_mode else False
                        print("Starting the robot in auto mode...")

                    elif event.key == pygame.K_UP:
                        left_LPWM = 0
                        right_LPWM = 0
                        left_RPWM = self.MAX_PWM
                        right_RPWM = self.MAX_PWM
                        print("Moving forward...")

                    elif event.key == pygame.K_DOWN:
                        left_LPWM = self.MAX_PWM
                        right_LPWM = self.MAX_PWM
                        left_RPWM = 0
                        right_RPWM = 0
                        print("Moving backward...")

                    elif event.key == pygame.K_LEFT:
                        left_LPWM = self.MAX_PWM
                        right_LPWM = 0
                        left_RPWM = 0
                        right_RPWM = self.MAX_PWM
                        print("Moving left...")

                    elif event.key == pygame.K_RIGHT:
                        left_LPWM = 0
                        right_LPWM = self.MAX_PWM
                        left_RPWM = self.MAX_PWM
                        right_RPWM = 0
                        print("Moving right...")

                    elif event.key == pygame.K_SPACE:
                        self.running = False
                        print("Stopping the robot...")

            # print(right_RPWM, right_LPWM, left_RPWM, left_LPWM)
            if self.running:
                self.send_data(right_RPWM, right_LPWM, left_RPWM, left_LPWM)
            else:
                self.send_data(0, 0, 0, 0)
            time.sleep(0.1)

    def start(self):
        try:
            self.run()
            # self.run_fluct()
        except:
            self.send_data(0, 0, 0, 0)
        return

    
if __name__ == "__main__":
    pygame.init()
    pygame.display.set_caption("Robot Control")

    screen = pygame.display.set_mode((200, 200))
    clock = pygame.time.Clock()
    controller = Controller()

    try:
        controller.start()
    except KeyboardInterrupt:
        pass
    except:
        print(str(sys.exc_info()))

    pygame.quit()
