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
RS485_DE_PIN = 24

PPR = 20
UART_DEVICE = '/dev/serial0'
UART_BAUDRATE = 507000

# Глобальные переменные
counter = 0
angle_rad = 0.0

# Предварительно вычисленные константы
ANGLE_MULTIPLIER = 2 * math.pi / PPR

class EncoderReader:
    def __init__(self, pigpio, a_pin, b_pin, z_pin):
        self._pi = pigpio
        self._cb_a = None
        self._cb_z = None
        self._running = False
        self._a_pin = a_pin
        self._b_pin = b_pin
        self._z_pin = z_pin

    def start(self):
        global counter
        if not self._pi.connected:
            raise RuntimeError("pigpio daemon is not running")

        # Минимальная настройка пинов
        self._pi.set_mode(self._a_pin, pigpio.INPUT)
        self._pi.set_pull_up_down(self._a_pin, pigpio.PUD_UP)
        self._pi.set_mode(self._b_pin, pigpio.INPUT)
        self._pi.set_pull_up_down(self._b_pin, pigpio.PUD_UP)
        self._pi.set_mode(self._z_pin, pigpio.INPUT)
        self._pi.set_pull_up_down(self._z_pin, pigpio.PUD_UP)

        # Минимальный фильтр дребезга
        self._pi.set_glitch_filter(self._a_pin, 50)
        self._pi.set_glitch_filter(self._b_pin, 50)
        self._pi.set_glitch_filter(self._z_pin, 50)

        # Обработчики прерываний
        self._cb_a = self._pi.callback(self._a_pin, pigpio.EITHER_EDGE, self._handle_A)
        self._cb_z = self._pi.callback(self._z_pin, pigpio.RISING_EDGE, self._handle_Z)

        self._running = True

    def stop(self):
        self._running = False
        if self._cb_a:
            self._cb_a.cancel()
        if self._cb_z:
            self._cb_z.cancel()
        if self._pi:
            self._pi.stop()

    def _handle_A(self, gpio, level, tick):
        global counter
        if level == pigpio.TIMEOUT or not self._running:
            return

        a = self._pi.read(self._a_pin)
        b = self._pi.read(self._b_pin)

        if level == 1:
            counter += 1 if b == 0 else -1
        else:
            counter += 1 if b == 1 else -1

    def _handle_Z(self, gpio, level, tick):
        global counter
        if level == 1 and self._running:
            counter = 0

class RS485Transmitter:
    def __init__(self, pigpio, device, baudrate, rs485_de_pin):
        self._device = device
        self._baudrate = baudrate
        self._serial_port = None
        self._pi = pigpio
        self._running = False
        self._rs485_de_pin = rs485_de_pin
        
        #Создаем пакет
        self._packet = bytearray(120)
        self._packet[0] = 0x65
        self._packet[118] = 0x45
        self._packet[119] = 0xCF

    def start(self):
        try:
            if not self._pi.connected:
                raise RuntimeError("pigpio daemon is not running")

            self._pi.set_mode(self._rs485_de_pin, pigpio.OUTPUT)
            self._pi.write(self._rs485_de_pin, 0)

            self._serial_port = serial.Serial(
                port=self._device,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001,
                write_timeout=0.001
            )

            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()

            # Включаем передачу
            self._pi.write(self._rs485_de_pin, 1)

            self._running = True

        except Exception as e:
            raise

    def stop(self):
        self._running = False
        if self._pi:
            self._pi.write(self._rs485_de_pin, 0)
            self._pi.stop()
        if self._serial_port and self._serial_port.is_open:
            self._serial_port.close()

    def send_packet(self, counter_value):
        """Отправка пакета с минимальными операциями"""
        if not self._running:
            return False

        try:

            # Вычисляем точный угол на основе счетчика
            angle = counter_value * ANGLE_MULTIPLIER
            #angle_bytes = struct.pack('<f', angle)
            self._packet[55:59] = struct.pack('<f', angle)

            # Контрольная сумма
            checksum = sum(self._packet[55:59], 0x65)
            self._packet[117] = 0xFF - (0xFF & checksum)

            # Отправляем пакет
            self._serial_port.write(self._packet)
            
            # Делаем небольшую задержку
            time.sleep(0.0023)

            return True
        except Exception as e:
            # В случае ошибки отключаем передачу
            try:
                self._pi.write(self._rs485_de_pin, 0)
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

    pi_instance = pigpio.pi()
    
    # Инициализация энкодера
    encoder = EncoderReader(pigpio=pi_instance, a_pin=A_PIN, b_pin=B_PIN, z_pin=Z_PIN)
    try:
        encoder.start()
    except Exception as e:
        print(f"Ошибка инициализации энкодера: {e}")
        return
    
    # Инициализация RS-485
    rs485 = RS485Transmitter(pi_instance, UART_DEVICE, UART_BAUDRATE, RS485_DE_PIN)
    try:
        rs485.start()
    except Exception as e:
        print(f"Ошибка инициализации RS-485: {e}")
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
