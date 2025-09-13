#!/usr/bin/env python3
"""
RS-485 передача с максимальным приоритетом и изоляцией CPU
"""

import pigpio
import math
import time
import struct
import serial
import os
import ctypes
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
            # Привязка к конкретному CPU ядру (например, ядро 3)
            os.sched_setaffinity(0, {3})
            
            # Установка политики планирования FIFO с максимальным приоритетом
            param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            
            # Установка высокого nice значения
            os.nice(-20)
            
        except PermissionError:
            print("Требуются права root для установки реального времени")
            # Падаем, если нет прав
            raise
        except Exception as e:
            print(f"Ошибка установки приоритета: {e}")
    
    def _transmit_loop(self):
        """Основной цикл передачи"""
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
            
            # Настройка UART
            ser = serial.Serial(
                port=UART_DEVICE,
                baudrate=UART_BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Предварительно созданный шаблон пакета
            packet_template = bytearray(120)
            packet_template[0] = 0x65
            packet_template[118] = 0x45
            packet_template[119] = 0xCF
            
            # Отключаем garbage collection для этого потока
            import gc
            gc.disable()
            
            # Основной цикл передачи
            interval = 0.003  # 3 мс
            next_time = time.monotonic()
            
            while self.running:
                current_time = time.monotonic()
                
                if current_time >= next_time:
                    # Создаем пакет
                    packet = packet_template.copy()  # Копируем шаблон
                    
                    # Заполняем данные
                    angle = counter * ANGLE_MULTIPLIER
                    angle_bytes = struct.pack('<f', angle)
                    packet[55:59] = angle_bytes
                    
                    checksum = 0x65 + sum(angle_bytes)
                    packet[117] = 0xFF - (0xFF & checksum)
                    
                    # Передача
                    pi.write(RS485_DE_PIN, 1)  # Включаем передатчик
                    ser.write(packet)
                    ser.flush()  # Ждем завершения передачи
                    pi.write(RS485_DE_PIN, 0)  # Выключаем передатчик
                    
                    next_time += interval
                
                # Короткая пауза для экономии CPU
                time.sleep(0.0001)
                
        except Exception as e:
            print(f"Ошибка в цикле передачи: {e}")
        finally:
            # Гарантированно выключаем передатчик
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
        self.pi.set_mode(A_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(A_PIN, pigpio.PUD_UP)
        self.pi.set_mode(B_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(B_PIN, pigpio.PUD_UP)
        self.pi.set_mode(Z_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(Z_PIN, pigpio.PUD_UP)
        
        self.pi.set_glitch_filter(A_PIN, 50)
        self.pi.set_glitch_filter(B_PIN, 50)
        self.pi.set_glitch_filter(Z_PIN, 50)
        
        self.cb_a = self.pi.callback(A_PIN, pigpio.EITHER_EDGE, self._handle_A)
        self.cb_z = self.pi.callback(Z_PIN, pigpio.RISING_EDGE, self._handle_Z)
    
    def _handle_A(self, gpio, level, tick):
        global counter
        b = self.pi.read(B_PIN)
        if level == 1:
            counter += 1 if b == 0 else -1
        else:
            counter += 1 if b == 1 else -1
            
    def _handle_Z(self, gpio, level, tick):
        global counter
        counter = 0
        
    def cleanup(self):
        if self.pi.connected:
            self.pi.stop()

def main():
    global counter
    
    print("Запуск системы передачи данных...")
    print("Для остановки нажмите Ctrl+C")
    
    # Инициализация энкодера
    encoder = EncoderReader()
    
    # Запуск передатчика
    transmitter = RealTimeTransmitter()
    
    try:
        transmitter.start()
        
        # Главный цикл просто ждет
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nОстановка системы...")
    finally:
        transmitter.stop()
        encoder.cleanup()

if __name__ == "__main__":
    # Запускаем от root для реального приоритета
    if os.geteuid() != 0:
        print("Запустите с правами root: sudo python3 script.py")
        exit(1)
    
    main()