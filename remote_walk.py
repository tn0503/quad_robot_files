from machine import Pin, Timer, PWM
import time

# ******************
# ***** Action *****
# ******************
STOP = 0
STEP = 1
FWRD = 2
BWRD = 3
LTRN = 4
RTRN = 5
LEFT = 6
RGHT = 7
EXILE = 8
WALK = 9

SV_FREQ = 50.0  # サーボ信号周波数
MAX_DUTY = 65025.0 # 周期内の分割数
MIN_SV_PULSE = 0.6  # 最小パルス幅　0°
MAX_SV_PULSE = 2.4  # 最大パルス幅 180°

correction = [14,-14,4, 0,0,6, -25,-12,4, 20,-4,-8]
servo = []
temp_angle = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]
step = [\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 3],\
[90,100,110, 90, 90, 90, 90, 90, 90, 90, 80, 70, 3],\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 8],\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 3],\
[90, 90, 90, 90, 80, 70, 90,100,110, 90, 90, 90, 3],\
[90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 8]\
]
fwrd = [\
[90,105, 70, 90, 86, 95, 90, 94, 85, 90, 75,110, 3],\
[90,100,110, 90, 82,100, 90, 98, 80, 90, 80, 70, 3],\
[90, 90, 90, 90, 78,105, 90,102, 75, 90, 90, 90, 8],\
[90, 94, 85, 90, 75,110, 90,105, 70, 90, 86, 95, 3],\
[90, 98, 80, 90, 80, 70, 90,100,110, 90, 82,100, 3],\
[90,102, 75, 90, 90, 90, 90, 90, 90, 90, 78,105, 8]\
]
bwrd = [\
[90,102, 75, 90, 90, 90, 90, 90, 90, 90, 78,105, 3],\
[90, 98, 80, 90, 80, 70, 90,100,110, 90, 82,100, 3],\
[90, 94, 85, 90, 75,110, 90,105, 70, 90, 86, 95, 8],\
[90, 90, 90, 90, 78,105, 90,102, 75, 90, 90, 90, 3],\
[90,100,110, 90, 82,100, 90, 98, 80, 90, 80, 70, 3],\
[90,105, 70, 90, 86, 95, 90, 94, 85, 90, 75,110, 8]\
]
ltrn = [\
[90-10, 90, 90, 90+ 5, 90, 90, 90+ 5, 90, 90, 90-10, 90, 90, 3],\
[90+10,100,110, 90+ 0, 90, 90, 90+ 0, 90, 90, 90+10, 80, 70, 3],\
[90+10, 90, 90, 90- 5, 90, 90, 90- 5, 90, 90, 90+10, 90, 90, 8],\
[90+ 5, 90, 90, 90-10, 90, 90, 90-10, 90, 90, 90+ 5, 90, 90, 3],\
[90+ 0, 90, 90, 90+10, 80, 70, 90+10,100,110, 90+ 0, 90, 90, 3],\
[90- 5, 90, 90, 90+10, 90, 90, 90+10, 90, 90, 90- 5, 90, 90, 8]\
]
rtrn = [\
[90+10, 90, 90, 90- 5, 90, 90, 90- 5, 90, 90, 90+10, 90, 90, 3],\
[90-10,100,110, 90- 0, 90, 90, 90- 0, 90, 90, 90-10, 80, 70, 3],\
[90-10, 90, 90, 90+ 5, 90, 90, 90+ 5, 90, 90, 90-10, 90, 90, 8],\
[90- 5, 90, 90, 90+10, 90, 90, 90+10, 90, 90, 90- 5, 90, 90, 3],\
[90- 0, 90, 90, 90-10, 80, 70, 90-10,100,110, 90- 0, 90, 90, 3],\
[90+ 5, 90, 90, 90-10, 90, 90, 90-10, 90, 90, 90+ 5, 90, 90, 8]\
]
left = [\
[90-10, 90, 90, 90+ 5, 90, 90, 90- 5, 90, 90, 90+10, 90, 90, 3],\
[90+10,100,110, 90+ 0, 90, 90, 90- 0, 90, 90, 90-10, 80, 70, 3],\
[90+10, 90, 90, 90- 5, 90, 90, 90+ 5, 90, 90, 90-10, 90, 90, 8],\
[90+ 5, 90, 90, 90-10, 90, 90, 90+10, 90, 90, 90- 5, 90, 90, 3],\
[90+ 0, 90, 90, 90+10, 80, 70, 90-10,100,110, 90- 0, 90, 90, 3],\
[90- 5, 90, 90, 90+10, 90, 90, 90-10, 90, 90, 90+ 5, 90, 90, 8]\
]
rght = [\
[90+10, 90, 90, 90- 5, 90, 90, 90+ 5, 90, 90, 90-10, 90, 90, 3],\
[90-10,100,110, 90- 0, 90, 90, 90+ 0, 90, 90, 90+10, 80, 70, 3],\
[90-10, 90, 90, 90+ 5, 90, 90, 90- 5, 90, 90, 90+10, 90, 90, 8],\
[90- 5, 90, 90, 90+10, 90, 90, 90-10, 90, 90, 90+ 5, 90, 90, 3],\
[90- 0, 90, 90, 90-10, 80, 70, 90+10,100,110, 90+ 0, 90, 90, 3],\
[90+ 5, 90, 90, 90-10, 90, 90, 90+10, 90, 90, 90- 5, 90, 90, 8]\
]
walk = [\
[90,105,70,90,83,98,90,91,87,90,78,105, 3],\
[90,100,110,90,81,101,90,93,85,90,76,107, 3],\
[90,90,90,90,80,103,90,95,83,90,75,110, 3],\
[90,91,87,90,78,105,90,96,81,90,75,110, 3],\
[90,93,85,90,76,107,90,98,78,90,80,70, 3],\
[90,95,83,90,75,110,90,100,76,90,90,90, 3],\
[90,96,81,90,75,110,90,101,74,90,88,92, 3],\
[90,98,78,90,80,70,90,103,72,90,86,94, 3],\
[90,100,76,90,90,90,90,105,70,90,85,96, 3],\
[90,101,74,90,88,92,90,105,70,90,83,98, 3],\
[90,103,72,90,86,94,90,100,110,90,81,101, 3],\
[90,105,70,90,85,96,90,90,90,90,80,103, 3]\
]
exile = [\
[ 75, 90, 90, 75, 90, 90, 75, 90, 90, 75, 90, 90, 20],\
[ 90,105,120, 90, 75, 60, 90, 75, 60, 90,105,120, 20],\
[105, 90, 90,105, 90, 90,105, 90, 90,105, 90, 90, 20],\
[ 90, 75, 60, 90,105,120, 90,105,120, 90, 75, 60, 20]\
]
div_counter = 0
key_frame = 0
next_key_frame = 1
rows = 0
action = []
action_mode = STOP
servo_flag = False
tim = Timer()

# 30Hzのタイマー割り込み
def tick(timer):
    global servo_flag
    servo_flag = True
   
tim.init(freq=30, mode=Timer.PERIODIC, callback=tick)

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
    
# ******************
# ***** LED *****
# ******************
led = Pin(25, Pin.OUT)
led.value(0)

# ******************
# ***** Remote *****
# ******************
# 設定
remote_in = Pin(28, Pin.IN, Pin.PULL_UP)

# 変数
rm_received = False  #信号受信完了した
digit = 0            #受信データの桁
rm_state = 0         #信号受信状況
rm_code = 0          #コード全体(32bit)
prev_micros = 0      #時間計測用

# 割り込み
def int_handler(pin):
    global rm_state
    global prev_micros
    global digit
    global rm_code
    global rm_received
    
    if rm_state != 0:
        width = time.ticks_us() - prev_micros    #時間間隔を計算
        if width > 10000:
            rm_state = 0    #長すぎ
        prev_micros = time.ticks_us()
        
    if rm_state == 0:    #信号未達
        prev_micros = time.ticks_us()    #現在時刻(microseconds)を記憶
        rm_state = 1    #最初のLOW->HIGH信号を検出した
        rm_code = 0
        digit = 0
    elif rm_state == 1:    #最初のHIGH状態
        if width > 9500 or width < 8500:    #リーダーコード(9ms)ではない
            rm_state = 0
        else:
            rm_state = 2    #HIGH->LOWで9ms検出
    elif rm_state == 2:    #9ms検出した
        if width > 5000 or width < 4000:    #リーダーコード(4.5ms)ではない
            rm_state = 0
        else:
            rm_state = 3    #LOW->HIGHで4.5ms検出
    elif rm_state == 3:    #4.5ms検出した
        if width > 700 or width < 400:
            rm_state = 0
        else:
            rm_state = 4    #HIGH->LOWで0.56ms検出した
    elif rm_state == 4:    #0.56ms検出した
        if width > 1800 or width < 400:   #LOW期間(2.25-0.56)msより長い or (1.125-0.56)msより短い
            rm_state = 0
        else:
            if width > 1000:    #LOW期間長い -> 1
                rm_code |= (1 << digit)
            else:             #LOW期間短い -> 0
                rm_code &= ~(1 << digit)
            digit += 1  #次のbit
           
            if digit > 31:   #完了
                rm_received = True
                return
            rm_state = 3    #次のHIGH->LOWを待つ
   
remote_in.irq(trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = int_handler)

while True:
    # *** Remote ***
    if rm_received == True:    #リモコン受信した
        rm_received = False    #初期化
        rm_state = 0      #初期化
        #図とは左右が逆であることに注意
        custom_code = rm_code & 0xffff   #下16bitがcustomCode
        data_code = (rm_code & 0xff0000) >> 16   #下16bitを捨てたあとの下8bitがdataCode
        inv_data_code = (rm_code & 0xff000000) >> 24    #下24bitを捨てたあとの下8bitがinvDataCode
        if (data_code + inv_data_code) == 0xff:    #反転確認
            print("data_code="+str(data_code))
            if data_code == 248:
                led.value(1)
            elif data_code == 120:
                led.value(0)
            elif data_code == 32:
                if action_mode == STEP:
                    action_mode = STOP
                else:
                    action_mode = STEP
                    action.clear()
                    action = step.copy()
                    rows = len(step)
            elif data_code == 160:
                action_mode = FWRD
                action.clear()
                action = fwrd.copy()
                rows = len(fwrd)
            elif data_code == 0:
                action_mode = BWRD
                action.clear()
                action = bwrd.copy()
                rows = len(bwrd)
            elif data_code == 177:
                action_mode = LTRN
                action.clear()
                action = ltrn.copy()
                rows = len(ltrn)
            elif data_code == 33:
                action_mode = RTRN
                action.clear()
                action = rtrn.copy()
                rows = len(rtrn)
            elif data_code == 16:
                action_mode = LEFT
                action.clear()
                action = left.copy()
                rows = len(left)
            elif data_code == 128:
                action_mode = RGHT
                action.clear()
                action = rght.copy()
                rows = len(rght)
            elif data_code == 88:
                action_mode = EXILE
                action.clear()
                action = exile.copy()
                rows = len(exile)
            elif data_code == 216:
                action_mode = WALK
                action.clear()
                action = walk.copy()
                rows = len(walk)
                                
    # *** Servo ***            
    if servo_flag == True:
        servo_flag = False
        if action_mode != STOP:
            # キーフレームを更新
            div_counter += 1
            if div_counter >= action[key_frame][12]:
                div_counter = 0
                key_frame = next_key_frame
                next_key_frame += 1
                if next_key_frame > rows-1:
                    next_key_frame = 0
            # 角度計算
            for i in range(12):
                temp_angle[i] = action[key_frame][i] +\
                    (action[next_key_frame][i] - action[key_frame][i])\
                    * div_counter / action[key_frame][12]
        else:
            for i in range(12):
                temp_angle[i] = 90
        # サーボ駆動
        for i in range(12):
            servo[i].duty_u16(get_pulse_width(int(temp_angle[i]) + correction[i]))
        #time.sleep(0.003) # 0.03秒待ち