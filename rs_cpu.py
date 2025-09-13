#!/usr/bin/env python3
"""
RS-485 передача с минимальной задержкой между пакетами
"""

import pigpio
import math
import time
import struct
import serial
import os
import threading

# Настройка пинов
A_PIN = 17
B_PIN = 22
Z_PIN = 27
RS485_DE_PIN = 23

PPR = 20
UART_DEVICE = '/dev/serial0'
UART_BAUDRATE = 507000

# Глобальные переменные
counter = 0
ANGLE_MULTIPLIER = 2 * math.pi / PPR

class EncoderReader:
    def init(self):
        self.pi = None
        self.cb_a = None
        self.cb_z = None
        self.setup_encoder()
        
    def setup_encoder(self):
        """Настройка энкодера"""
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                print("Ошибка подключения к pigpio daemon")
                return
            
            pins = [A_PIN, B_PIN, Z_PIN]
            for pin in pins:
                self.pi.set_mode(pin, pigpio.INPUT)
                self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
                self.pi.set_glitch_filter(pin, 50)
            
            self.cb_a = self.pi.callback(A_PIN, pigpio.EITHER_EDGE, self._handle_A)
            self.cb_z = self.pi.callback(Z_PIN, pigpio.RISING_EDGE, self._handle_Z)
            
        except Exception as e:
            print(f"Ошибка инициализации энкодера: {e}")
    
    def _handle_A(self, gpio, level, tick):
        global counter
        if self.pi is None:
            return
        b = self.pi.read(B_PIN)
        counter += 1 if (level == 1 and b == 0) or (level == 0 and b == 1) else -1
            
    def _handle_Z(self, gpio, level, tick):
        global counter
        counter = 0
        
    def cleanup(self):
        """Безопасное освобождение ресурсов"""
        try:
            if self.cb_a:
                self.cb_a.cancel()
            if self.cb_z:
                self.cb_z.cancel()
            if self.pi and self.pi.connected:
                self.pi.stop()
        except Exception as e:
            print(f"Ошибка при cleanup: {e}")
class HighSpeedTransmitter:
    def init(self):
        self.running = False
        self.pi = None
        self.ser = None
        
    def start(self):
        """Запуск высокоскоростной передачи"""
        global counter
        
        try:
            # Инициализация pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                print("Ошибка подключения к pigpio daemon")
                return False
            
            # Инициализация UART
            self.ser = serial.Serial(
                port=UART_DEVICE,
                baudrate=UART_BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0
            )
            
            # Настройка пинов RS-485
            self.pi.set_mode(RS485_DE_PIN, pigpio.OUTPUT)
            self.pi.write(RS485_DE_PIN, 0)
            
            # Предварительный шаблон пакета
            packet_template = bytearray(120)
            packet_template[0] = 0x65
            packet_template[118] = 0x45
            packet_template[119] = 0xCF
            
            self.running = True
            
            while self.running:
                # Быстрое создание пакета
                packet = bytearray(packet_template)  # Копируем шаблон
                
                # Заполняем данные угла
                angle = counter * ANGLE_MULTIPLIER
                angle_bytes = struct.pack('<f', angle)
                packet[55:59] = angle_bytes
                
                # Контрольная сумма
                checksum = 0x65 + sum(angle_bytes)
                packet[117] = 0xFF - (0xFF & checksum)
                
                # Сверхбыстрая передача
                self.pi.write(RS485_DE_PIN, 1)  # Включаем передатчик
                self.ser.write(packet)          # Отправляем данные
                self.pi.write(RS485_DE_PIN, 0)  # Выключаем передатчик
                    
            return True
                    
        except Exception as e:
            print(f"Ошибка в передатчике: {e}")
            return False
            
    def stop(self):
        """Остановка передачи"""
        self.running = False
        self.cleanup()
        
    def cleanup(self):
        """Безопасное освобождение ресурсов"""
        try:
            if self.pi and self.pi.connected:
                self.pi.write(RS485_DE_PIN, 0)  # Гарантированно выключаем
                self.pi.stop()
        except:
            pass
            
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass

def main():
    global counter
    
    # Инициализация энкодера
    encoder = EncoderReader()
    
    # Запуск передатчика
    transmitter = HighSpeedTransmitter()
    
    try:
        # Запускаем передачу в основном потоке
        transmitter.start()
        
    except KeyboardInterrupt:
        print("\nОстановка по запросу пользователя...")
    except Exception as e:
        print(f"Ошибка в main: {e}")
    finally:
        # Гарантированная очистка
        transmitter.stop()
        encoder.cleanup()
        print("Система остановлена")
if __name__ == "__main__":
    main()