import sys
import time

import navio.pwm
import navio.util

navio.util.check_apm()

PWM_speed = 2
PWM_steer = 1
SERVO_MIN = 1 #ms
SERVO_MAX = 2 #ms


pwm = navio.pwm.PWM(0)
pwm.initialize()
pwm.set_period(50)
pwm.enable()

pwm1 = navio.pwm.PWM(1)
pwm1.initialize()
pwm1.set_period(50)
pwm1.enable()

pwm2 = navio.pwm.PWM(2)
pwm2.initialize()
pwm2.set_period(50)
pwm2.enable()

#pwm2.set_duty_cycle(SERVO_MIN)
#time.sleep(0.5)
#pwm2.set_duty_cycle(SERVO_MAX)
#pwm1.set_duty_cycle(SERVO_MIN)
#time.sleep(0.5)
#pwm1.set_duty_cycle(SERVO_MAX)
