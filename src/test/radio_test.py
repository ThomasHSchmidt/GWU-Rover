import sys, time

import navio.rcinput
import navio.pwm
import navio.util

navio.util.check_apm()

rcin = navio.rcinput.RCInput()
    
PWM_speed = 0
PWM_steer = 1
SERVO_MIN = 1 #ms
SERVO_MAX = 2 #ms


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


pwm_speed = navio.pwm.PWM(PWM_speed)
pwm_steer = navio.pwm.PWM(PWM_steer)
pwm_speed.initialize()
pwm_speed.set_period(50)
pwm_speed.enable()
pwm_steer.initialize()
pwm_steer.set_period(50)
pwm_steer.enable()

while (True):
        speed = int(rcin.read(1))
        steer = int(rcin.read(0))
        pwm_steer.set_duty_cycle(map(steer,800,2100,1,2))
        pwm_speed.set_duty_cycle(map(speed,800,2100,1,2))
