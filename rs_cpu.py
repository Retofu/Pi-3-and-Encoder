#!/usr/bin/env python3
"""
RS-485 передача с изолированным CPU и максимальным приоритетом
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
angle_rad = 0.0
ANGLE_MULTIPLIER = 2 * math.pi / PPR

# Приоритет реального времени
SCHED_FIFO = 1
SCHED_RR = 2

class RealTimeThread(threading.Thread):
    def init(self):
        super().init()
        self.daemon = True
        self.running = False
        
    def run(self):
        # Привязываем поток к изолированному CPU 3
        os.sched_setaffinity(0, {3})
        
        # Устанавливаем максимальный приоритет реального времени
        param = os.sched_param(os.sched_get_priority_max(SCHED_FIFO))
        os.sched_setscheduler(0, SCHED_FIFO, param)
        
        # Отключаем все возможные прерывания для этого потока
        self._disable_interrupts()
        
        self.running = True
        self._main_loop()
    
    def _disable_interrupts(self):
        """Минимизируем прерывания для текущего потока"""
        try:
            # Отключаем GC
            import gc
            gc.disable()
            
            # Блокируем все сигналы
            import signal
            signal.pthread_sigmask(signal.SIG_BLOCK, range(1, signal.NSIG))
            
        except:
            pass
    
    def _main_loop(self):
        """Основной цикл в реальном времени"""
        global counter
        
        # Инициализация оборудования
        pi = pigpio.pi()
        if not pi.connected:
            return
            
        # Настройка пинов
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
        
        # Тайминг
        interval = 0.003  # 3 мс
        next_time = time.monotonic()
        
        try:
            while self.running:
                current_time = time.monotonic()
                
                if current_time >= next_time:
                    # Создаем пакет
                    packet = packet_template[:]
                    angle = counter * ANGLE_MULTIPLIER
                    angle_bytes = struct.pack('<f', angle)
                    packet[55:59] = angle_bytes
                    
                    checksum = 0x65 + sum(angle_bytes)
                    packet[117] = 0xFF - (0xFF & checksum)
                    
                    # Передача
                    pi.write(RS485_DE_PIN, 1)
                    ser.write(packet)
                    ser.flush()
                    pi.write(RS485_DE_PIN, 0)
                    
                    next_time += interval
                else:
                    # Микро-сон для точного тайминга
                    time.sleep(max(0, next_time - current_time - 0.0001))
                    
        finally:
            pi.write(RS485_DE_PIN, 0)
            ser.close()
            pi.stop()

class EncoderReader:
    def init(self):
        self.pi = pigpio.pi()
        self.setup_encoder()
        
    def setup_encoder(self):
        """Настройка энкодера на отдельном ядре"""
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

def main():
    global counter
    
    # Инициализация энкодера
    encoder = EncoderReader()
    
    # Запуск реального времени потока
    rt_thread = RealTimeThread()
    rt_thread.start()
    
    try:
        # Главный поток просто ждет
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        rt_thread.running = False
        rt_thread.join()
        
    finally:
        if encoder.pi.connected:
            encoder.pi.stop()

if __name__ == "__main__":
    main()