#!/usr/bin/env python3
"""
Простой тест энкодера для диагностики проблем
"""

import pigpio
import time

# Настройка пинов энкодера
A_PIN = 17  # Фаза A (GPIO17, pin 11)
B_PIN = 22  # Фаза B (GPIO22, pin 12) 
Z_PIN = 27  # Фаза Z (GPIO27, pin 13)

counter = 0

def handle_A(gpio, level, tick):
    """Обработчик фазы A"""
    global counter
    if level == pigpio.TIMEOUT:
        return
    
    try:
        a = pi.read(A_PIN)
        b = pi.read(B_PIN)
        
        # Логика энкодера
        if level == 1:  # RISING edge на A
            if b == 0:
                counter += 1
            else:
                counter -= 1
        else:  # FALLING edge на A
            if b == 1:
                counter += 1
            else:
                counter -= 1
        
        print(f"Энкодер: A={a}, B={b}, Счетчик={counter}")
    except Exception as e:
        print(f"Ошибка: {e}")

def main():
    global pi
    
    print("=== Тест энкодера ===")
    print("Подключите энкодер к пинам:")
    print(f"  A_PIN = GPIO{A_PIN} (физический pin 11)")
    print(f"  B_PIN = GPIO{B_PIN} (физический pin 15)")
    print(f"  Z_PIN = GPIO{Z_PIN} (физический pin 13)")
    print("Нажмите Enter для продолжения...")
    input()
    
    pi = pigpio.pi()
    if not pi.connected:
        print("Ошибка: pigpio daemon не запущен")
        print("Запустите: sudo systemctl start pigpiod")
        return
    
    # Настройка пинов
    pi.set_mode(A_PIN, pigpio.INPUT)
    pi.set_pull_up_down(A_PIN, pigpio.PUD_UP)
    pi.set_mode(B_PIN, pigpio.INPUT)
    pi.set_pull_up_down(B_PIN, pigpio.PUD_UP)
    pi.set_mode(Z_PIN, pigpio.INPUT)
    pi.set_pull_up_down(Z_PIN, pigpio.PUD_UP)
    
    # Фильтр дребезга
    pi.set_glitch_filter(A_PIN, 200)
    pi.set_glitch_filter(B_PIN, 200)
    pi.set_glitch_filter(Z_PIN, 200)
    
    # Диагностика состояния пинов
    print("\nДиагностика энкодера:")
    print(f"  Пин A (GPIO{A_PIN}): {pi.read(A_PIN)}")
    print(f"  Пин B (GPIO{B_PIN}): {pi.read(B_PIN)}")
    print(f"  Пин Z (GPIO{Z_PIN}): {pi.read(Z_PIN)}")
    
    # Обработчик прерываний
    cb_a = pi.callback(A_PIN, pigpio.EITHER_EDGE, handle_A)
    
    print("\nКрутите энкодер и наблюдайте за изменением счетчика...")
    print("Нажмите Ctrl+C для остановки")
    
    changes_count = 0
    max_counter = counter
    min_counter = counter
    last_counter = counter
    
    try:
        while True:
            if counter != last_counter:
                changes_count += 1
                max_counter = max(max_counter, counter)
                min_counter = min(min_counter, counter)
                last_counter = counter
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\nФинальный счетчик: {counter}")
        print(f"Количество изменений: {changes_count}")
        print(f"Максимальное значение: {max_counter}")
        print(f"Минимальное значение: {min_counter}")
        if changes_count >= 5:
            print("✓ Энкодер работает корректно!")
        else:
            print("✗ Энкодер не реагирует на вращение")
    finally:
        cb_a.cancel()
        pi.stop()

if __name__ == "__main__":
    main()
