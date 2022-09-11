import sys
import time

import navio.pwm
import navio.util

navio.util.check_apm()

PWM_speed = 2
PWM_steer = 1
SERVO_MIN = 1 #ms
SERVO_MAX = 2 #ms

with navio.pwm.PWM(3) as pwm:
    pwm.set_period(50)
    pwm.enable()

    while (True):
        pwm.set_duty_cycle(SERVO_MIN)
        time.sleep(0.5)
        pwm.set_duty_cycle(SERVO_MAX)
        break