#!/usr/bin/env python3
"""
RS-485 передача через аппаратный UART - МИНИМАЛЬНАЯ ВЕРСИЯ
Максимальная оптимизация для достижения 3 мс цикла
"""

import pigpio
import math
import time
import struct
import serial
import os

# Настройка пинов энкодера
A_PIN = 17
B_PIN = 22
Z_PIN = 27

# Настройка пинов RS-485
RS485_DE_PIN = 23

PPR = 20
UART_DEVICE = '/dev/serial0'
UART_BAUDRATE = 507000

# Глобальные переменные
counter = 0
angle_rad = 0.0

# Предварительно вычисленные константы
ANGLE_MULTIPLIER = 2 * math.pi / PPR

class EncoderReader:
    def __init__(self):
        self.pi = None
        self.cb_a = None
        self.cb_z = None
        self.running = False
        
    def start(self):
        global counter
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running")
            
        # Минимальная настройка пинов
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
        self.running = False
        if self.cb_a:
            self.cb_a.cancel()
        if self.cb_z:
            self.cb_z.cancel()
        if self.pi:
            self.pi.stop()
        
    def _handle_A(self, gpio, level, tick):
        global counter
        if level == pigpio.TIMEOUT or not self.running:
            return
        
        a = self.pi.read(A_PIN)
        b = self.pi.read(B_PIN)
        
        if level == 1:
            counter += 1 if b == 0 else -1
        else:
            counter += 1 if b == 1 else -1
            
    def _handle_Z(self, gpio, level, tick):
        global counter
        if level == 1 and self.running:
            counter = 0

class RS485Transmitter:
    def __init__(self, device=UART_DEVICE, baudrate=UART_BAUDRATE):
        self.device = device
        self.baudrate = baudrate
        self.serial_port = None
        self.pi = None
        self.running = False
        
    def start(self):
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
        self.running = False
        if self.pi:
            self.pi.write(RS485_DE_PIN, 0)
            self.pi.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def send_packet(self, counter_value):
        """Отправка пакета с минимальными операциями"""
        if not self.running:
            return False
        
        try:
            # Создаем пакет динамически для точного угла
            packet = bytearray(120)
            packet[0] = 0x65
            packet[118] = 0x45
            packet[119] = 0xCF
            
            # Вычисляем точный угол на основе счетчика
            angle = counter_value * ANGLE_MULTIPLIER
            angle_bytes = struct.pack('<f', angle)
            packet[55:59] = angle_bytes
            
            # Контрольная сумма
            checksum = 0x65 + sum(angle_bytes)
            packet[117] = 0xFF - (0xFF & checksum)
            
            # Включаем передачу
            self.pi.write(RS485_DE_PIN, 1)
            
            # Очищаем буферы перед отправкой
            self.serial_port.reset_output_buffer()
            
            # Отправляем пакет
            self.serial_port.write(packet)
            time.sleep(0.0023)
            
            # Отключаем передачу
            self.pi.write(RS485_DE_PIN, 0)
            
            return True
        except Exception as e:
            # В случае ошибки отключаем передачу
            try:
                self.pi.write(RS485_DE_PIN, 0)
            except:
                pass
            return False

def main():
    global counter
    
    # Устанавливаем высокий приоритет процессу
    try:
        os.nice(-20)  # Максимальный приоритет
    except:
        pass
    
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
            # Отправляем пакет напрямую по счетчику
            rs485.send_packet(counter)
            
    except KeyboardInterrupt:
        pass
    finally:
        rs485.stop()
        encoder.stop()

if __name__ == "__main__":
    main()
