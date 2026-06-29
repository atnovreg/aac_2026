import time
import serial
import pygame

PORT = "/dev/ttyACM0"   # Arduino Mega 포트. 안 되면 /dev/ttyUSB0로 변경
BAUD = 9600

DEADZONE = 0.25

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("조이스틱이 연결되지 않았습니다.")
    ser.close()
    raise SystemExit

joy = pygame.joystick.Joystick(0)
joy.init()

print("Joystick:", joy.get_name())
print("Start control. Ctrl+C to stop.")

last_cmd = None

def send_cmd(cmd):
    global last_cmd
    if cmd != last_cmd:
        ser.write(cmd.encode())
        print("CMD:", cmd)
        last_cmd = cmd

try:
    while True:
        pygame.event.pump()

        steer = joy.get_axis(0)
        throttle = joy.get_axis(1)

        forward = throttle < -DEADZONE
        reverse = throttle > DEADZONE
        left = steer < -DEADZONE
        right = steer > DEADZONE

        if forward and left:
            cmd = 'G'
        elif forward and right:
            cmd = 'I'
        elif reverse and left:
            cmd = 'H'
        elif reverse and right:
            cmd = 'J'
        elif forward:
            cmd = 'F'
        elif reverse:
            cmd = 'B'
        elif left:
            cmd = 'L'
        elif right:
            cmd = 'R'
        else:
            cmd = 'S'

        send_cmd(cmd)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stop")
    send_cmd('S')

finally:
    ser.close()
    pygame.quit()

