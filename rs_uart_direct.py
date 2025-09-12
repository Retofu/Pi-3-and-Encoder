#!/usr/bin/env python3
"""
RS-485 передача через прямой доступ к UART - МАКСИМАЛЬНАЯ СКОРОСТЬ
Обход pyserial для минимальных задержек
"""

import pigpio
import math
import time
import struct
import os
import fcntl
import termios

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

class DirectUART:
    def __init__(self, device=UART_DEVICE, baudrate=UART_BAUDRATE):
        self.device = device
        self.baudrate = baudrate
        self.fd = None
        self.pi = None
        self.running = False
        
        # Предварительно создаем все возможные пакеты
        self.packets = []
        self._create_all_packets()
        
    def _create_all_packets(self):
        """Создаем все возможные пакеты заранее"""
        for i in range(PPR):
            packet = bytearray(120)
            packet[0] = 0x65
            packet[118] = 0x45
            packet[119] = 0xCF
            
            # Упаковываем угол
            angle = i * ANGLE_MULTIPLIER
            angle_bytes = struct.pack('<f', angle)
            packet[55:59] = angle_bytes
            
            # Контрольная сумма
            checksum = 0x65 + sum(angle_bytes)
            packet[117] = 0xFF - (0xFF & checksum)
            
            self.packets.append(packet)
        
    def start(self):
        try:
            # Открываем UART напрямую
            self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            
            # Настраиваем UART
            tty = termios.tcgetattr(self.fd)
            
            # Устанавливаем скорость
            speed = getattr(termios, f'B{self.baudrate}', termios.B115200)
            termios.cfsetispeed(tty, speed)
            termios.cfsetospeed(tty, speed)
            
            # Настраиваем параметры
            tty[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            tty[3] = 0
            tty[4] = 0
            tty[5] = 0
            tty[6][termios.VMIN] = 0
            tty[6][termios.VTIME] = 0
            
            termios.tcsetattr(self.fd, termios.TCSANOW, tty)
            
            # Инициализация pigpio для DE пина
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon is not running")
            
            self.pi.set_mode(RS485_DE_PIN, pigpio.OUTPUT)
            self.pi.write(RS485_DE_PIN, 0)
            
            self.running = True
            
        except Exception as e:
            raise
    
    def stop(self):
        self.running = False
        if self.pi:
            self.pi.write(RS485_DE_PIN, 0)
            self.pi.stop()
        if self.fd:
            os.close(self.fd)
    
    def send_packet(self, counter_value):
        """Отправка пакета через прямой доступ к UART"""
        if not self.running:
            return False
        
        try:
            # Получаем предварительно созданный пакет
            packet = self.packets[counter_value % PPR]
            
            # Включаем передачу
            self.pi.write(RS485_DE_PIN, 1)
            
            # Отправляем пакет напрямую
            os.write(self.fd, packet)
            
            # Отключаем передачу
            self.pi.write(RS485_DE_PIN, 0)
            
            return True
        except:
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
    
    # Инициализация прямого UART
    uart = DirectUART()
    try:
        uart.start()
    except Exception as e:
        encoder.stop()
        return
    
    try:
        while True:
            # Отправляем пакет напрямую по счетчику
            uart.send_packet(counter)
            
    except KeyboardInterrupt:
        pass
    finally:
        uart.stop()
        encoder.stop()

if __name__ == "__main__":
    main()
