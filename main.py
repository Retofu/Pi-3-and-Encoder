import pigpio
import math
import time

A_PIN = 17
B_PIN = 22
Z_PIN = 27
PPR = 1200

counter = 0

pi = pigpio.pi()                   # подключение к демону
if not pi.connected:
    raise RuntimeError("pigpio daemon is not running")

# Входы с подтяжкой вверх
pi.set_mode(A_PIN, pigpio.INPUT)
pi.set_pull_up_down(A_PIN, pigpio.PUD_UP)
pi.set_mode(B_PIN, pigpio.INPUT)
pi.set_pull_up_down(B_PIN, pigpio.PUD_UP)
pi.set_mode(Z_PIN, pigpio.INPUT)
pi.set_pull_up_down(Z_PIN, pigpio.PUD_UP)

# Небольшой фильтр дребезга (микросекунды)
pi.set_glitch_filter(A_PIN, 200)
pi.set_glitch_filter(B_PIN, 200)
pi.set_glitch_filter(Z_PIN, 200)

def handle_A(gpio, level, tick):
    global counter
    if level == pigpio.TIMEOUT:
        return
    # Квадратура: сравниваем текущие уровни A и B
    a = pi.read(A_PIN)
    b = pi.read(B_PIN)
    counter += 1 if a == b else -1

def handle_Z(gpio, level, tick):
    global counter
    if level == 1:  # RISING
        counter = 0

cb_a = pi.callback(A_PIN, pigpio.EITHER_EDGE, handle_A)
cb_z = pi.callback(Z_PIN, pigpio.RISING_EDGE, handle_Z)

try:
    while True:
        angle_rad = (counter % PPR) * (2 * math.pi / PPR)
        print(f"Угол: {angle_rad:.3f} рад, Счетчик: {counter}")
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    cb_a.cancel()
    cb_z.cancel()
    pi.stop()