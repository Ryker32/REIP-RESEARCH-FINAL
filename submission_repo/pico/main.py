"""
REIP Robot - Pi Pico Motor Controller
MicroPython firmware for motor PWM and encoder reading

Pin Configuration (matches clanker hardware):
- Motor L: GP18, GP19 (via DRV8833 AIN1/AIN2)
- Motor R: GP20, GP21 (via DRV8833 BIN1/BIN2)
- Encoder L: GP3 (A), GP4 (B)
- Encoder R: GP7 (A), GP8 (B)
- UART TX: GP0 -> Pi Zero RX
- UART RX: GP1 <- Pi Zero TX

Supports both text and JSON protocols:
Text: PING, ENC, MOT,L,R, STOP, RST
JSON: {"cmd":"motor","left":100,"right":100}
"""

import time
from machine import Pin, PWM, UART

# ============================================================================
# PIN CONFIGURATION - Matches your working hardware
# ============================================================================

# Motor PWM pins (to DRV8833)
MOTOR_L_IN1 = 18  # GP18 -> DRV AIN1
MOTOR_L_IN2 = 19  # GP19 -> DRV AIN2
MOTOR_R_IN1 = 20  # GP20 -> DRV BIN1
MOTOR_R_IN2 = 21  # GP21 -> DRV BIN2

# Encoder pins
ENC_L_A = 3       # GP3 <- Left encoder Ch.A
ENC_L_B = 4       # GP4 <- Left encoder Ch.B
ENC_R_A = 7       # GP7 <- Right encoder Ch.A
ENC_R_B = 8       # GP8 <- Right encoder Ch.B

# UART to Pi Zero
UART_TX = 0       # GP0 -> Pi Zero RX
UART_RX = 1       # GP1 <- Pi Zero TX

# PWM frequency
PWM_FREQ = 1000  # not audible

# Motor direction inversion (set True if wheel spins backwards)
INVERT_LEFT = True   # Your left motor needs inversion
INVERT_RIGHT = False

# ============================================================================
# GLOBALS
# ============================================================================

left_count = 0
right_count = 0

# ============================================================================
# MOTOR CONTROL
# ============================================================================


def setup_motors():
    """Initialize motor PWM pins"""
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
    """
    Set motor speeds.
    speed_l, speed_r: -100 to 100 (percentage)
    """
    # Apply inversion
    if INVERT_LEFT:
        speed_l = -speed_l
    if INVERT_RIGHT:
        speed_r = -speed_r
    
    # Clamp to valid range
    speed_l = max(-100, min(100, speed_l))
    speed_r = max(-100, min(100, speed_r))
    
    # Left motor
    if speed_l >= 0:
        pwms['l1'].duty_u16(int(speed_l * 655.35))
        pwms['l2'].duty_u16(0)
    else:
        pwms['l1'].duty_u16(0)
        pwms['l2'].duty_u16(int(-speed_l * 655.35))
    
    # Right motor
    if speed_r >= 0:
        pwms['r1'].duty_u16(int(speed_r * 655.35))
        pwms['r2'].duty_u16(0)
    else:
        pwms['r1'].duty_u16(0)
        pwms['r2'].duty_u16(int(-speed_r * 655.35))


def stop_motors(pwms):
    """Stop both motors"""
    for p in pwms.values():
        p.duty_u16(0)


# ============================================================================
# ENCODERS
# ============================================================================


def setup_encoders():
    """Initialize encoder pins with interrupts"""
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
    """Reset encoder counts to zero"""
    global left_count, right_count
    left_count = 0
    right_count = 0


# ============================================================================
# COMMAND PROCESSING
# ============================================================================


def process_text_command(cmd, pwms, uart):
    """
    Process simple text commands (your working protocol)
    Commands: PING, ENC, MOT,L,R, STOP, RST
    """
    global left_count, right_count
    
    cmd = cmd.strip().upper()
    
    if cmd == "PING":
        uart.write("PONG\n")
    
    elif cmd == "ENC":
        uart.write(f"{left_count},{right_count}\n")
    
    elif cmd.startswith("MOT,"):
        try:
            parts = cmd.split(",")
            l = float(parts[1])
            r = float(parts[2])
            set_motor(pwms, l, r)
            uart.write("OK\n")
        except:
            uart.write("ERR\n")
    
    elif cmd == "STOP":
        stop_motors(pwms)
        uart.write("OK\n")
    
    elif cmd == "RST":
        reset_encoders()
        uart.write("OK\n")
    
    else:
        # Unknown command - try JSON fallback
        return False
    
    return True


def process_json_command(line, pwms, uart):
    """
    Process JSON commands (for future REIP integration)
    """
    global left_count, right_count
    
    try:
        import json
        cmd = json.loads(line)
        response = {"ok": True}
        
        cmd_type = cmd.get("cmd", "")
        
        if cmd_type == "motor":
            left = cmd.get("left", 0)
            right = cmd.get("right", 0)
            set_motor(pwms, left, right)
        
        elif cmd_type == "stop":
            stop_motors(pwms)
        
        elif cmd_type == "encoders":
            response["left"] = left_count
            response["right"] = right_count
        
        elif cmd_type == "reset_encoders":
            reset_encoders()
        
        elif cmd_type == "ping":
            response["pong"] = time.ticks_ms()
        
        else:
            response["ok"] = False
            response["error"] = f"Unknown: {cmd_type}"
        
        uart.write(json.dumps(response) + "\n")
        return True
        
    except:
        return False


# ============================================================================
# MAIN
# ============================================================================


def main():
    # Initialize hardware
    pwms = setup_motors()
    encoders = setup_encoders()
    
    # Initialize UART
    uart = UART(0, baudrate=115200, tx=Pin(UART_TX), rx=Pin(UART_RX))
    
    # Status LED
    led = Pin(25, Pin.OUT)
    led_state = False
    last_blink = time.ticks_ms()
    
    # Command timeout (auto-stop if no commands for 500ms)
    last_cmd_time = time.ticks_ms()
    motors_active = False
    
    print("REIP Pico Ready")
    
    # Input buffer
    buffer = ""
    
    # Main loop
    while True:
        # Check for incoming commands
        if uart.any():
            try:
                data = uart.read()
                if data:
                    buffer += data.decode()
            except:
                buffer = ""
            
            # Process complete lines
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                
                if line:
                    last_cmd_time = time.ticks_ms()
                    
                    # Try text command first, then JSON
                    if not process_text_command(line, pwms, uart):
                        if not process_json_command(line, pwms, uart):
                            uart.write("ERR:UNKNOWN\n")
                    
                    # Track if motors might be active
                    if "MOT" in line.upper() or '"motor"' in line.lower():
                        motors_active = True
        
        # Safety timeout - stop motors if no commands for 500ms
        if motors_active and time.ticks_diff(time.ticks_ms(), last_cmd_time) > 500:
            stop_motors(pwms)
            motors_active = False
        
        # Blink LED (heartbeat)
        if time.ticks_diff(time.ticks_ms(), last_blink) > 500:
            led_state = not led_state
            led.value(led_state)
            last_blink = time.ticks_ms()
        
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
