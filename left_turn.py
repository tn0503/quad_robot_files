from machine import Pin, PWM
import time

SV_FREQ = 50.0  # サーボ信号周波数
MAX_DUTY = 65025.0 # 周期内の分割数
MIN_SV_PULSE = 0.6  # 最小パルス幅　0°
MAX_SV_PULSE = 2.4  # 最大パルス幅 180°

correction = [14,-14,4, 0,0,6, -25,-12,4, 20,-4,-8]
servo = []
temp_angle = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]

angle = [\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90],\
[90,100,110, 90, 90, 90, 90, 90, 90, 90, 80, 70],\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90],\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90],\
[90, 90, 90, 90, 80, 70, 90,100,110, 90, 90, 90],\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]\
]

divide = 3
div_counter = 0
key_frame = 0
next_key_frame = 1

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
    # キーフレームを更新
    div_counter += 1
    if div_counter >= divide:
        div_counter = 0
        key_frame = next_key_frame
        next_key_frame += 1
        if next_key_frame > 5:
            next_key_frame = 0
    # 角度計算
    for i in range(12):
        temp_angle[i] = angle[key_frame][i] +\
            (angle[next_key_frame][i] - angle[key_frame][i])\
            * div_counter / divide
    # サーボ駆動
    for i in range(12):
        servo[i].duty_u16(get_pulse_width(int(temp_angle[i]) + correction[i]))
    time.sleep(0.03) # 0.03秒待ち