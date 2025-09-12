#!/usr/bin/env python3
"""
Тест RS-485 передачи через GPIO пины
"""

import pigpio
import time

# Настройка пинов RS-485
RS485_TX_PIN = 14  # GPIO14 (pin 8) - передача данных
RS485_DE_PIN = 23  # GPIO23 (pin 16) - управление направлением (DE/RE)

RS485_BAUDRATE = 9600

def send_test_byte(pi, byte_value):
    """Отправка одного байта через RS-485"""
    bit_delay = 1.0 / RS485_BAUDRATE
    
    try:
        # Start bit (0)
        pi.write(RS485_TX_PIN, 0)
        time.sleep(bit_delay)
        
        # Data bits (LSB first)
        for bit in range(8):
            bit_value = (byte_value >> bit) & 1
            pi.write(RS485_TX_PIN, bit_value)
            time.sleep(bit_delay)
        
        # Stop bit (1)
        pi.write(RS485_TX_PIN, 1)
        time.sleep(bit_delay)
        
        return True
    except Exception as e:
        print(f"Ошибка отправки байта: {e}")
        return False

def main():
    print("=== Тест RS-485 GPIO ===")
    print("Подключение к логическому анализатору:")
    print(f"  TX = GPIO{RS485_TX_PIN} (pin 8)")
    print(f"  DE = GPIO{RS485_DE_PIN} (pin 16)")
    print("Нажмите Enter для начала теста...")
    input()
    
    pi = pigpio.pi()
    if not pi.connected:
        print("Ошибка: pigpio daemon не запущен")
        print("Запустите: sudo systemctl start pigpiod")
        return
    
    # Настройка пинов
    pi.set_mode(RS485_TX_PIN, pigpio.OUTPUT)
    pi.set_mode(RS485_DE_PIN, pigpio.OUTPUT)
    
    # Начальные состояния
    pi.write(RS485_TX_PIN, 1)  # Idle состояние
    pi.write(RS485_DE_PIN, 0)  # Отключить передачу
    
    print("Отправка тестовых данных...")
    
    try:
        for i in range(10):
            print(f"Отправка пакета #{i+1}")
            
            # Включаем передачу
            pi.write(RS485_DE_PIN, 1)
            time.sleep(0.001)
            
            # Отправляем тестовый пакет: 0x65 0x00 0x00 ... 0x45 0xCF
            test_packet = [0x65] + [0x00] * 116 + [0x45, 0xCF]
            
            for byte_value in test_packet:
                if not send_test_byte(pi, byte_value):
                    break
            
            # Отключаем передачу
            pi.write(RS485_DE_PIN, 0)
            
            print(f"  Пакет #{i+1} отправлен")
            time.sleep(0.1)  # Пауза между пакетами
            
    except KeyboardInterrupt:
        print("\nТест прерван")
    finally:
        pi.write(RS485_DE_PIN, 0)
        pi.write(RS485_TX_PIN, 1)
        pi.stop()
        print("Тест завершен")

if __name__ == "__main__":
    main()
