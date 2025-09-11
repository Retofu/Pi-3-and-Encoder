import RPi.GPIO as GPIO
import math
import time

# Настройка пинов
A_PIN = 17  # Фаза A (GPIO17, pin 11)
B_PIN = 22  # Фаза B (GPIO22)
Z_PIN = 27  # Фаза Z (GPIO27, pin 13)

PPR = 1200  # Разрешение энкодера (импульсов на оборот). Уточните в даташите вашего энкодера!

counter = 0  # Счетчик импульсов
angle_rad = 0.0  # Текущий угол в радианах

# Настройка GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(Z_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Обработчик прерывания для фазы A
def handle_encoder_A(channel):
    global counter
    if GPIO.input(A_PIN) == GPIO.input(B_PIN):
        counter += 1  # Вращение по часовой стрелке
    else:
        counter -= 1  # Вращение против часовой стрелки

# Обработчик прерывания для фазы Z (индексный импульс)
def handle_encoder_Z(channel):
    global counter
    counter = 0  # Сброс счетчика на абсолютный ноль

# Назначение прерываний
GPIO.add_event_detect(A_PIN, GPIO.RISING, callback=handle_encoder_A, bouncetime=2)
GPIO.add_event_detect(Z_PIN, GPIO.RISING, callback=handle_encoder_Z, bouncetime=2)
try:
    while True:
        # Расчет угла в радианах
        angle_rad = (counter % PPR) * (2 * math.pi / PPR)
        print(f"Угол: {angle_rad:.3f} рад, Счетчик: {counter}")
        time.sleep(0.01)  # Короткая задержка
except KeyboardInterrupt:
    GPIO.cleanup()