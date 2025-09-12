#!/usr/bin/env python3
"""
RS-485 передача через GPIO пины - версия без вывода в консоль для максимальной производительности
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

PPR = 20  # Разрешение энкодера (импульсов на оборот)
RS485_BAUDRATE = 500000  # Высокая скорость для достижения 2.75 мс

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
        
        # Минимальный фильтр дребезга
        self.pi.set_glitch_filter(A_PIN, 50)
        self.pi.set_glitch_filter(B_PIN, 50)
        self.pi.set_glitch_filter(Z_PIN, 50)
        
        # Обработчики прерываний
        self.cb_a = self.pi.callback(A_PIN, pigpio.EITHER_EDGE, self._handle_A)
        self.cb_z = self.pi.callback(Z_PIN, pigpio.RISING_EDGE, self._handle_Z)
        
        self.running = True
        
    def stop(self):
        """Остановка чтения энкодера"""
        self.running = False
        if self.cb_a:
            self.cb_a.cancel()
        if self.cb_z:
            self.cb_z.cancel()
        if self.pi:
            self.pi.stop()
        
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
        except:
            pass
            
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
            
        except Exception as e:
            raise
    
    def stop(self):
        """Остановка RS-485 интерфейса"""
        self.running = False
        if self.pi:
            # Отключаем передачу
            self.pi.write(RS485_DE_PIN, 0)
            self.pi.write(RS485_TX_PIN, 1)
            self.pi.stop()
    
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
        except:
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
            time.sleep(0.00005)  # Минимальная задержка (50 мкс)
            
            # Отправляем каждый байт
            for byte_value in packet:
                if not self.send_byte(byte_value):
                    return False
            
            # Отключаем передачу
            self.pi.write(RS485_DE_PIN, 0)
            return True
            
        except:
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
    
    print("=== RS-485 Silent Mode - Максимальная производительность ===")
    print("Версия без вывода в консоль для достижения максимальной скорости")
    print("Нажмите Ctrl+C для остановки")
    
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
        print("✓ RS-485 инициализирован")
    except Exception as e:
        print(f"Ошибка инициализации RS-485: {e}")
        encoder.stop()
        return
    
    print("Система запущена. Передача данных каждые 3 мс (без вывода в консоль)")
    
    packet_count = 0
    start_time_total = time.time()
    
    try:
        while True:
            # Обновление угла
            update_angle()
            
            # Создание пакета данных
            packet = rs485.create_data_packet(angle_rad)
            
            # Отправка пакета (без измерения времени для максимальной скорости)
            if rs485.send_packet(packet):
                packet_count += 1
                
                # Вывод статистики только каждые 10000 пакетов
                if packet_count % 10000 == 0:
                    elapsed_time = time.time() - start_time_total
                    packets_per_second = packet_count / elapsed_time
                    print(f"Пакетов отправлено: {packet_count}, Скорость: {packets_per_second:.1f} пакетов/сек")
            else:
                print("Ошибка отправки пакета")
                break
            
            # Пауза между пакетами (0.25 мс)
            time.sleep(0.00025)
            
    except KeyboardInterrupt:
        print(f"\nОстановка... Отправлено пакетов: {packet_count}")
    finally:
        rs485.stop()
        encoder.stop()

if __name__ == "__main__":
    main()
