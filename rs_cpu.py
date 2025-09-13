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

# Время передачи одного пакета (120 байт × 10 бит / 507000 бод)
PACKET_TRANSMIT_TIME = 120 * 10 / 507000  # ≈ 2.366 мс

class RealTimeTransmitter:
    def init(self):
        self.running = False
        self.thread = None
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._transmit_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
    
    def _set_realtime_priority(self):
        """Установка приоритета реального времени"""
        try:
            # Привязка к конкретному CPU ядру
            os.sched_setaffinity(0, {3})
            
            # Установка политики планирования FIFO
            param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            
        except Exception as e:
            print(f"Ошибка приоритета: {e}")
    
    def _transmit_loop(self):
        """Основной цикл передачи с минимальной задержкой"""
        global counter
        
        # Устанавливаем реальный time приоритет
        self._set_realtime_priority()
        
        # Инициализация pigpio и UART
        pi = pigpio.pi()
        if not pi.connected:
            return
        
        try:
            # Настройка пинов RS-485
            pi.set_mode(RS485_DE_PIN, pigpio.OUTPUT)
            pi.write(RS485_DE_PIN, 0)
            
            # Настройка UART с минимальными таймаутами
            ser = serial.Serial(
                port=UART_DEVICE,
                baudrate=UART_BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0001,  # Минимальный timeout
                write_timeout=0.0001,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Предварительно созданный шаблон пакета
            packet_template = bytearray(120)
            packet_template[0] = 0x65
            packet_template[118] = 0x45
            packet_template[119] = 0xCF
            
            # Отключаем garbage collection
            import gc
            gc.disable()
            
            # Основной цикл передачи - БЕЗ sleep!
            packet_count = 0
            start_time = time.monotonic()
            
            while self.running:
                # Создаем пакет
                packet = packet_template[:]  # Быстрое копирование
                
                # Заполняем данные
                angle = counter * ANGLE_MULTIPLIER
                angle_bytes = struct.pack('<f', angle)
                packet[55:59] = angle_bytes
                
                checksum = 0x65 + sum(angle_bytes)
                packet[117] = 0xFF - (0xFF & checksum)
                
                # Передача - БЕЗ ожидания!
                pi.write(RS485_DE_PIN, 1)  # Включаем передатчик
                ser.write(packet)          # Отправляем (неблокирующе)
                pi.write(RS485_DE_PIN, 0)  # Сразу выключаем передатчик
                
                packet_count += 1
                
                # Очень короткая пауза для соблюдения интервала
                # Используем busy-wait для точности
                if packet_count % 100 == 0:  # Каждые 100 пакетов проверяем время
                    elapsed = time.monotonic() - start_time
                    expected_time = packet_count * 0.003  # 3 мс на пакет
                if elapsed < expected_time:
                        # Корректируем скорость если отстаем
                        pass
                
        except Exception as e:
            print(f"Ошибка передачи: {e}")
        finally:
            try:
                pi.write(RS485_DE_PIN, 0)
                ser.close()
            except:
                pass
            pi.stop()

class EncoderReader:
    def init(self):
        self.pi = pigpio.pi()
        self.setup_encoder()
        
    def setup_encoder(self):
        """Настройка энкодера"""
        pins = [A_PIN, B_PIN, Z_PIN]
        for pin in pins:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            self.pi.set_glitch_filter(pin, 50)
        
        self.cb_a = self.pi.callback(A_PIN, pigpio.EITHER_EDGE, self._handle_A)
        self.cb_z = self.pi.callback(Z_PIN, pigpio.RISING_EDGE, self._handle_Z)
    
    def _handle_A(self, gpio, level, tick):
        global counter
        b = self.pi.read(B_PIN)
        counter += 1 if (level == 1 and b == 0) or (level == 0 and b == 1) else -1
            
    def _handle_Z(self, gpio, level, tick):
        global counter
        counter = 0
        
    def cleanup(self):
        if self.pi.connected:
            self.pi.stop()

def main():
    global counter
    
    print("Запуск высокоскоростной передачи...")
    
    # Инициализация энкодера
    encoder = EncoderReader()
    
    # Запуск передатчика (простая версия)
    transmitter = RealTimeTransmitter()
    
    try:
        # Замер производительности
        start_time = time.monotonic()
        packet_count = 0
        
        transmitter.start()
        
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        transmitter.running = False

if __name__ == "__main__":
   
    main()