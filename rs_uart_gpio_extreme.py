#!/usr/bin/env python3
"""
RS-485 передача через аппаратный UART на GPIO14/15 - ЭКСТРЕМАЛЬНО БЫСТРАЯ ВЕРСИЯ
МАКСИМАЛЬНАЯ ОПТИМИЗАЦИЯ ДЛЯ ДОСТИЖЕНИЯ 3 МС ЦИКЛА
"""

import pigpio
import math
import time
import struct
import serial

# Настройка пинов энкодера
A_PIN = 17  # Фаза A (GPIO17, pin 11)
B_PIN = 22  # Фаза B (GPIO22, pin 12) 
Z_PIN = 27  # Фаза Z (GPIO27, pin 13)

# Настройка пинов RS-485
RS485_DE_PIN = 23  # GPIO23 (pin 16) - управление направлением (DE/RE)

PPR = 20  # Разрешение энкодера (импульсов на оборот)

# Настройки UART
UART_DEVICE = '/dev/serial0'  # Аппаратный UART Raspberry Pi
UART_BAUDRATE = 507000  # Максимальная скорость по ТЗ

# Глобальные переменные для данных энкодера
counter = 0
angle_rad = 0.0

# Предварительно вычисленные константы
ANGLE_MULTIPLIER = 2 * math.pi / PPR  # 0.3141592653589793

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
        
        a = self.pi.read(A_PIN)
        b = self.pi.read(B_PIN)
        
        if level == 1:  # RISING edge на A
            counter += 1 if b == 0 else -1
        else:  # FALLING edge на A
            counter += 1 if b == 1 else -1
            
    def _handle_Z(self, gpio, level, tick):
        """Обработчик индекса Z"""
        global counter
        if level == 1 and self.running:  # RISING
            counter = 0

class RS485Transmitter:
    """Класс для передачи данных через RS-485 через аппаратный UART - ЭКСТРЕМАЛЬНО ОПТИМИЗИРОВАННЫЙ"""
    
    def __init__(self, device=UART_DEVICE, baudrate=UART_BAUDRATE):
        self.device = device
        self.baudrate = baudrate
        self.serial_port = None
        self.pi = None
        self.running = False
        
        # Предварительно создаем шаблон пакета
        self.packet_template = bytearray(120)
        self.packet_template[0] = 0x65
        self.packet_template[118] = 0x45
        self.packet_template[119] = 0xCF
        
        # Предварительно создаем пакеты для всех возможных углов
        self.precomputed_packets = {}
        self._precompute_packets()
        
    def _precompute_packets(self):
        """Предварительное вычисление пакетов для всех возможных углов"""
        for i in range(PPR):
            angle = i * ANGLE_MULTIPLIER
            packet = self.packet_template[:]
            
            # Упаковываем угол
            angle_bytes = struct.pack('<f', angle)
            packet[55:59] = angle_bytes
            
            # Быстрое вычисление контрольной суммы
            checksum = 0x65 + sum(angle_bytes)
            packet[117] = 0xFF - (0xFF & checksum)
            
            self.precomputed_packets[i] = packet
        
    def start(self):
        """Инициализация RS-485 интерфейса через UART"""
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon is not running")
            
            self.pi.set_mode(RS485_DE_PIN, pigpio.OUTPUT)
            self.pi.write(RS485_DE_PIN, 0)
            
            self.serial_port = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001,
                write_timeout=0.001
            )
            
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.running = True
            
        except Exception as e:
            raise
    
    def stop(self):
        """Остановка RS-485 интерфейса"""
        self.running = False
        if self.pi:
            self.pi.write(RS485_DE_PIN, 0)
            self.pi.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def get_packet(self, counter_value):
        """Получение предварительно вычисленного пакета"""
        return self.precomputed_packets[counter_value % PPR]
    
    def send_packet(self, packet):
        """Отправка пакета данных через RS-485"""
        if not self.running:
            return False
        
        try:
            self.pi.write(RS485_DE_PIN, 1)
            self.serial_port.write(packet)
            self.serial_port.flush()
            self.pi.write(RS485_DE_PIN, 0)
            return True
        except:
            return False

def main():
    """Основная функция - ЭКСТРЕМАЛЬНО БЫСТРАЯ ВЕРСИЯ"""
    global counter
    
    # Инициализация энкодера
    encoder = EncoderReader()
    try:
        encoder.start()
    except Exception as e:
        return
    
    # Инициализация RS-485
    rs485 = RS485Transmitter()
    try:
        rs485.start()
    except Exception as e:
        encoder.stop()
        return
    
    try:
        while True:
            # Получаем предварительно вычисленный пакет
            packet = rs485.get_packet(counter)
            
            # Отправляем пакет
            rs485.send_packet(packet)
            
            # Пауза между пакетами (0.25 мс)
            # time.sleep(0.00025)
            
    except KeyboardInterrupt:
        pass
    finally:
        rs485.stop()
        encoder.stop()

if __name__ == "__main__":
    main()
