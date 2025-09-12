#!/usr/bin/env python3
"""
RS-485 передача через GPIO пины для логического анализатора
"""

import pigpio
import math
import time
import struct

# Настройка пинов энкодера
A_PIN = 17  # Фаза A (GPIO17, pin 11)
B_PIN = 22  # Фаза B (GPIO22, pin 12) 
Z_PIN = 27  # Фаза Z (GPIO27, pin 13)

# Настройка пинов RS-485
RS485_TX_PIN = 14  # GPIO14 (pin 8) - передача данных
RS485_RX_PIN = 15  # GPIO15 (pin 10) - прием данных  
RS485_DE_PIN = 23  # GPIO23 (pin 16) - управление направлением (DE/RE)

PPR = 1200  # Разрешение энкодера (импульсов на оборот)
RS485_BAUDRATE = 9600

# Глобальные переменные для данных энкодера
counter = 0
angle_rad = 0.0

class EncoderReader:
    """Класс для чтения данных с энкодера"""
    
    def __init__(self):
        self.pi = None
        self.cb_a = None
        self.cb_z = None
        self.running = False
        
    def start(self):
        """Инициализация и запуск чтения энкодера"""
        global counter
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running")
            
        # Настройка пинов
        self.pi.set_mode(A_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(A_PIN, pigpio.PUD_UP)
        self.pi.set_mode(B_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(B_PIN, pigpio.PUD_UP)
        self.pi.set_mode(Z_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(Z_PIN, pigpio.PUD_UP)
        
        # Фильтр дребезга
        self.pi.set_glitch_filter(A_PIN, 200)
        self.pi.set_glitch_filter(B_PIN, 200)
        self.pi.set_glitch_filter(Z_PIN, 200)
        
        # Диагностика состояния пинов
        print("Диагностика энкодера:")
        print(f"  Пин A (GPIO{A_PIN}): {self.pi.read(A_PIN)}")
        print(f"  Пин B (GPIO{B_PIN}): {self.pi.read(B_PIN)}")
        print(f"  Пин Z (GPIO{Z_PIN}): {self.pi.read(Z_PIN)}")
        
        # Обработчики прерываний
        self.cb_a = self.pi.callback(A_PIN, pigpio.EITHER_EDGE, self._handle_A)
        self.cb_z = self.pi.callback(Z_PIN, pigpio.RISING_EDGE, self._handle_Z)
        
        self.running = True
        print("Энкодер инициализирован")
        
    def stop(self):
        """Остановка чтения энкодера"""
        self.running = False
        if self.cb_a:
            self.cb_a.cancel()
        if self.cb_z:
            self.cb_z.cancel()
        if self.pi:
            self.pi.stop()
        print("Энкодер остановлен")
        
    def _handle_A(self, gpio, level, tick):
        """Обработчик фазы A"""
        global counter
        if level == pigpio.TIMEOUT or not self.running:
            return
        try:
            a = self.pi.read(A_PIN)
            b = self.pi.read(B_PIN)
            
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
        except Exception as e:
            print(f"Ошибка в обработчике A: {e}")
            
    def _handle_Z(self, gpio, level, tick):
        """Обработчик индекса Z"""
        global counter
        if level == 1 and self.running:  # RISING
            counter = 0

class RS485Transmitter:
    """Класс для передачи данных через RS-485 через GPIO пины"""
    
    def __init__(self, baudrate=RS485_BAUDRATE):
        self.baudrate = baudrate
        self.pi = None
        self.running = False
        self.bit_delay = 1.0 / baudrate  # Задержка между битами
        
    def start(self):
        """Инициализация RS-485 интерфейса через GPIO"""
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon is not running")
            
            # Настройка пинов RS-485
            self.pi.set_mode(RS485_TX_PIN, pigpio.OUTPUT)
            self.pi.set_mode(RS485_RX_PIN, pigpio.INPUT)
            self.pi.set_mode(RS485_DE_PIN, pigpio.OUTPUT)
            
            # Устанавливаем начальные состояния
            self.pi.write(RS485_TX_PIN, 1)  # Высокий уровень на TX (idle)
            self.pi.write(RS485_DE_PIN, 0)  # Отключить передачу (DE=0)
            
            self.running = True
            print(f"RS-485 инициализирован через GPIO: TX={RS485_TX_PIN}, RX={RS485_RX_PIN}, DE={RS485_DE_PIN}")
            print(f"Скорость: {self.baudrate} bps, Задержка бита: {self.bit_delay*1000:.2f} мс")
            
        except Exception as e:
            print(f"Ошибка инициализации RS-485 GPIO: {e}")
            raise
    
    def stop(self):
        """Остановка RS-485 интерфейса"""
        self.running = False
        if self.pi:
            # Отключаем передачу
            self.pi.write(RS485_DE_PIN, 0)
            self.pi.write(RS485_TX_PIN, 1)
            self.pi.stop()
        print("RS-485 остановлен")
    
    def send_byte(self, byte_value):
        """Отправка одного байта через RS-485"""
        if not self.running or not self.pi:
            return False
        
        try:
            # Start bit (0)
            self.pi.write(RS485_TX_PIN, 0)
            time.sleep(self.bit_delay)
            
            # Data bits (LSB first)
            for bit in range(8):
                bit_value = (byte_value >> bit) & 1
                self.pi.write(RS485_TX_PIN, bit_value)
                time.sleep(self.bit_delay)
            
            # Stop bit (1)
            self.pi.write(RS485_TX_PIN, 1)
            time.sleep(self.bit_delay)
            
            return True
        except Exception as e:
            print(f"Ошибка отправки байта: {e}")
            return False
    
    def create_data_packet(self, angle_rad):
        """
        Создание пакета данных размером 120 байт
        Байт 0: 0x65
        Байты 1-55 и 60-116: 0
        Байты 56-59: угол в радианах (float32, little-endian)
        Байт 117: контрольная сумма CS = 0xFF - (0xFF & Σᵢ СДᵢ)
        Байт 118: 0x45
        Байт 119: 0xCF
        """
        packet = bytearray(120)
        
        # Байт 0: 0x65
        packet[0] = 0x65
        
        # Байты 1-55: заполняем нулями (индексы 1-55)
        for i in range(1, 56):
            packet[i] = 0
        
        # Байты 56-59: угол в радианах как float32 (little-endian)
        angle_bytes = struct.pack('<f', angle_rad)
        packet[55:59] = angle_bytes  # Байты 56-59 (индексы 55-58)
        
        # Байты 60-116: заполняем нулями (индексы 59-116)
        for i in range(59, 117):
            packet[i] = 0
        
        # Вычисление контрольной суммы CS = 0xFF - (0xFF & Σᵢ СДᵢ)
        # Суммируем все байты от 0 до 116 (HDR и DATA)
        checksum = 0
        for i in range(117):  # Байты 0-116
            checksum += packet[i]
        
        # CS = 0xFF - (0xFF & checksum)
        packet[117] = 0xFF - (0xFF & checksum)
        
        # Байт 118: 0x45
        packet[118] = 0x45
        
        # Байт 119: 0xCF
        packet[119] = 0xCF
        
        return packet
    
    def send_packet(self, packet):
        """Отправка пакета данных через RS-485"""
        if not self.running:
            return False
        
        try:
            # Включаем передачу
            self.pi.write(RS485_DE_PIN, 1)
            time.sleep(0.001)  # Небольшая задержка для стабилизации
            
            # Отправляем каждый байт
            for byte_value in packet:
                if not self.send_byte(byte_value):
                    return False
            
            # Отключаем передачу
            self.pi.write(RS485_DE_PIN, 0)
            return True
            
        except Exception as e:
            print(f"Ошибка отправки пакета: {e}")
            return False

def update_angle():
    """Обновление угла в радианах"""
    global counter, angle_rad
    
    # Расчет угла в радианах
    angle_rad = (counter % PPR) * (2 * math.pi / PPR)
    
    # Если counter отрицательный, нормализуем его
    if counter < 0:
        angle_rad = (PPR + (counter % PPR)) * (2 * math.pi / PPR)

def main():
    """Основная функция"""
    global counter, angle_rad
    
    print("=== Raspberry Pi 3 Encoder + RS-485 GPIO Transmitter ===")
    print("Подключение к логическому анализатору:")
    print(f"  TX = GPIO{RS485_TX_PIN} (pin 8)")
    print(f"  RX = GPIO{RS485_RX_PIN} (pin 10)")
    print(f"  DE = GPIO{RS485_DE_PIN} (pin 16)")
    
    # Инициализация энкодера
    encoder = EncoderReader()
    try:
        encoder.start()
        print("✓ Энкодер инициализирован")
    except Exception as e:
        print(f"Ошибка инициализации энкодера: {e}")
        return
    
    # Инициализация RS-485
    rs485 = RS485Transmitter()
    try:
        rs485.start()
    except Exception as e:
        print(f"Ошибка инициализации RS-485: {e}")
        encoder.stop()
        return
    
    print("Система запущена. Передача данных каждые 3 мс. Нажмите Ctrl+C для остановки")
    
    packet_count = 0
    
    try:
        while True:
            # Обновление угла
            update_angle()
            
            # Создание пакета данных
            packet = rs485.create_data_packet(angle_rad)
            
            # Отправка пакета
            if rs485.send_packet(packet):
                packet_count += 1
                
                # Вывод информации о переданном пакете (каждый пакет)
                print(f"Пакет #{packet_count}: Угол={angle_rad:.3f} рад, Счетчик={counter}, "
                      f"Байты 56-59={packet[55:59].hex()}, CS={packet[117]:02x}, "
                      f"Заголовок={packet[0]:02x}{packet[118]:02x}{packet[119]:02x}")
            else:
                print("Ошибка отправки пакета")
            
            # Дополнительная диагностика каждые 1000 пакетов
            if packet_count % 1000 == 0:
                print(f"Диагностика: Пакетов={packet_count}, Счетчик={counter}, Угол={angle_rad:.3f} рад")
            
            # Интервал 3 мс
            time.sleep(0.003)
            
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        rs485.stop()
        encoder.stop()

if __name__ == "__main__":
    main()
