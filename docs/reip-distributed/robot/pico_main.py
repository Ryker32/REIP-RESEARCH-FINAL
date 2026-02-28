"""
REIP Robot - Pi Pico Motor Controller
MicroPython firmware for motor PWM and encoder reading
(Already installed on your Picos - included here for reference)

Flash this to Pico via Thonny if needed.
"""

import time
from machine import Pin, PWM, UART

# Pin Configuration
MOTOR_L_IN1 = 18
MOTOR_L_IN2 = 19
MOTOR_R_IN1 = 20
MOTOR_R_IN2 = 21

ENC_L_A = 3
ENC_L_B = 4
ENC_R_A = 7
ENC_R_B = 8

UART_TX = 0
UART_RX = 1

PWM_FREQ = 1000
INVERT_LEFT = True
INVERT_RIGHT = False

left_count = 0
right_count = 0

def setup_motors():
    pwms = {
        'l1': PWM(Pin(MOTOR_L_IN1)),
        'l2': PWM(Pin(MOTOR_L_IN2)),
        'r1': PWM(Pin(MOTOR_R_IN1)),
        'r2': PWM(Pin(MOTOR_R_IN2)),
    }
    for p in pwms.values():
        p.freq(PWM_FREQ)
        p.duty_u16(0)
    return pwms

def set_motor(pwms, speed_l, speed_r):
    if INVERT_LEFT:
        speed_l = -speed_l
    if INVERT_RIGHT:
        speed_r = -speed_r
    
    speed_l = max(-100, min(100, speed_l))
    speed_r = max(-100, min(100, speed_r))
    
    if speed_l >= 0:
        pwms['l1'].duty_u16(int(speed_l * 655.35))
        pwms['l2'].duty_u16(0)
    else:
        pwms['l1'].duty_u16(0)
        pwms['l2'].duty_u16(int(-speed_l * 655.35))
    
    if speed_r >= 0:
        pwms['r1'].duty_u16(int(speed_r * 655.35))
        pwms['r2'].duty_u16(0)
    else:
        pwms['r1'].duty_u16(0)
        pwms['r2'].duty_u16(int(-speed_r * 655.35))

def stop_motors(pwms):
    for p in pwms.values():
        p.duty_u16(0)

def setup_encoders():
    global left_count, right_count
    left_count = 0
    right_count = 0
    
    enc_left_a = Pin(ENC_L_A, Pin.IN, Pin.PULL_UP)
    enc_left_b = Pin(ENC_L_B, Pin.IN, Pin.PULL_UP)
    enc_right_a = Pin(ENC_R_A, Pin.IN, Pin.PULL_UP)
    enc_right_b = Pin(ENC_R_B, Pin.IN, Pin.PULL_UP)
    
    def left_handler(pin):
        global left_count
        left_count += 1 if enc_left_b.value() else -1
    
    def right_handler(pin):
        global right_count
        right_count += 1 if enc_right_b.value() else -1
    
    enc_left_a.irq(trigger=Pin.IRQ_RISING, handler=left_handler)
    enc_right_a.irq(trigger=Pin.IRQ_RISING, handler=right_handler)
    
    return (enc_left_a, enc_left_b, enc_right_a, enc_right_b)

def reset_encoders():
    global left_count, right_count
    left_count = 0
    right_count = 0

def main():
    global left_count, right_count
    
    pwms = setup_motors()
    encoders = setup_encoders()
    uart = UART(0, baudrate=115200, tx=Pin(UART_TX), rx=Pin(UART_RX))
    
    led = Pin(25, Pin.OUT)
    led_state = False
    last_blink = time.ticks_ms()
    last_cmd_time = time.ticks_ms()
    motors_active = False
    
    print("REIP Pico Ready")
    
    buffer = ""
    
    while True:
        if uart.any():
            try:
                data = uart.read()
                if data:
                    buffer += data.decode()
            except:
                buffer = ""
            
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip().upper()
                
                if line:
                    last_cmd_time = time.ticks_ms()
                    
                    if line == "PING":
                        uart.write("PONG\n")
                    elif line == "ENC":
                        uart.write(f"{left_count},{right_count}\n")
                    elif line.startswith("MOT,"):
                        try:
                            parts = line.split(",")
                            l = float(parts[1])
                            r = float(parts[2])
                            set_motor(pwms, l, r)
                            uart.write("OK\n")
                            motors_active = True
                        except:
                            uart.write("ERR\n")
                    elif line == "STOP":
                        stop_motors(pwms)
                        motors_active = False
                        uart.write("OK\n")
                    elif line == "RST":
                        reset_encoders()
                        uart.write("OK\n")
        
        # Safety timeout
        if motors_active and time.ticks_diff(time.ticks_ms(), last_cmd_time) > 500:
            stop_motors(pwms)
            motors_active = False
        
        # Heartbeat LED
        if time.ticks_diff(time.ticks_ms(), last_blink) > 500:
            led_state = not led_state
            led.value(led_state)
            last_blink = time.ticks_ms()
        
        time.sleep_ms(1)

if __name__ == "__main__":
    main()
