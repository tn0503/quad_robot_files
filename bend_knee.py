from machine import Pin, PWM
import time

SV_FREQ = 50.0  # サーボ信号周波数
MAX_DUTY = 65025.0 # 周期内の分割数
MIN_SV_PULSE = 0.6  # 最小パルス幅　0°
MAX_SV_PULSE = 2.4  # 最大パルス幅 180°

correction = [14,-14,4, 0,0,6, -18,-12,4, 20,-4,-8]
servo = []
angle = [\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90],\
[90,100,110, 90, 80, 70, 90,100,110, 90, 80, 70]\
]

# パルス幅を計算する関数
def get_pulse_width(angle):
    pulse_ms = MIN_SV_PULSE + (MAX_SV_PULSE - MIN_SV_PULSE) * angle / 180.0
    x = (int)(MAX_DUTY * (pulse_ms * SV_FREQ /1000.0))
    return x

# 全てのサーボを順番に駆動
for i in range(12):
    servo.append(PWM(Pin(11 - i)))
    servo[i].freq(50)
    servo[i].duty_u16(get_pulse_width(90 + correction[i]))

while True: # 繰り返し
    for i in range(12):
        servo[i].duty_u16(get_pulse_width(angle[0][i] + correction[i]))
    time.sleep(0.5) # 0.5秒待ち
    for i in range(12):
        servo[i].duty_u16(get_pulse_width(angle[1][i] + correction[i]))
    time.sleep(0.5) # 0.5秒待ち

