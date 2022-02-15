from machine import Pin, Timer
import time

# ***** Remote *****
# 設定
remote_in = Pin(28, Pin.IN, Pin.PULL_UP)

# 変数
rm_receiving = False
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
    global rm_receiving
    rm_receiving = True

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
    if rm_received == True:    #リモコン受信した
        print("rm_received")
        rm_received = False    #初期化
        rm_receiving = False
        rm_state = 0      #初期化
        #図とは左右が逆であることに注意
        custom_code = rm_code & 0xffff   #下16bitがcustomCode
        data_code = (rm_code & 0xff0000) >> 16   #下16bitを捨てたあとの下8bitがdataCode
        inv_data_code = (rm_code & 0xff000000) >> 24    #下24bitを捨てたあとの下8bitがinvDataCode
        if (data_code + inv_data_code) == 0xff:    #反転確認
            print("data_code="+str(data_code))
            