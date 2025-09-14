#!/usr/bin/env python3
"""
Объединенный скрипт для работы с энкодером, RS-485 и ModBus TCP
Полностью синхронная версия с threading для ModBus
"""

import pigpio
import math
import time
import struct
import serial
import os
import threading
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext

# Настройка пинов энкодера
A_PIN = 17
B_PIN = 22
Z_PIN = 27

# Настройка пинов RS-485
RS485_DE_PIN = 24

# Параметры энкодера
PPR = 20  # Из rs.py
UART_DEVICE = '/dev/serial0'
UART_BAUDRATE = 507000

# Настройка ModBus
MODBUS_PORT = 2502
MODBUS_UNIT_ID = 1

# Регистры ModBus
REG_ANGLE_RAD = 0      # Угол в радианах (float32, 2 регистра)
REG_ANGLE_DEG = 2      # Угол в градусах (float32, 2 регистра) 
REG_COUNTER = 4        # Счетчик импульсов (int32, 2 регистра)
REG_PPR = 6           # PPR энкодера (uint16, 1 регистр)

# Предварительно вычисленные константы
ANGLE_MULTIPLIER = 2 * math.pi / PPR

# Глобальные переменные (как в оригинале)
counter = 0
angle_rad = 0.0
angle_deg = 0.0

class Endian:
    Big = 0
    Little = 1

class BinaryPayloadBuilder:
    def __init__(self, byteorder=0, wordorder=0):
        self.data = []
        self.byteorder = byteorder

    def add_32bit_float(self, value):
        # Упаковываем float32 в big-endian (стандарт ModBus)
        packed = struct.pack('>f', value)
        # Разбиваем на два 16-битных регистра
        reg1 = int.from_bytes(packed[:2], 'big')
        reg2 = int.from_bytes(packed[2:], 'big')
        self.data.extend([reg1, reg2])

    def add_32bit_int(self, value):
        # Упаковываем int32 в big-endian (стандарт ModBus)
        packed = struct.pack('>i', value)
        # Разбиваем на два 16-битных регистра
        reg1 = int.from_bytes(packed[:2], 'big')
        reg2 = int.from_bytes(packed[2:], 'big')
        self.data.extend([reg1, reg2])

    def to_registers(self):
        return self.data

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

class ModbusDataStore:
    """Класс для обновления данных в ModBus регистрах"""
    
    def __init__(self, hr_block, ir_block):
        self.hr_block = hr_block  # Holding Registers
        self.ir_block = ir_block  # Input Registers
        
    def update_registers(self):
        """Обновление регистров данными энкодера"""
        global counter, angle_rad, angle_deg
        
        # Расчет углов
        angle_rad = (counter % PPR) * (2 * math.pi / PPR)
        angle_deg = math.degrees(angle_rad)
        
        # Упаковка float32 в 2 регистра (big-endian)
        builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        builder.add_32bit_float(angle_rad)
        angle_rad_data = builder.to_registers()
        
        builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        builder.add_32bit_float(angle_deg)
        angle_deg_data = builder.to_registers()
        
        # Упаковка int32 счетчика в 2 регистра
        builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        builder.add_32bit_int(counter)
        counter_data = builder.to_registers()
        
        # Обновление регистров в обоих блоках (Holding и Input)
        self.hr_block.setValues(REG_ANGLE_RAD, angle_rad_data)
        self.hr_block.setValues(REG_ANGLE_DEG, angle_deg_data)
        self.hr_block.setValues(REG_COUNTER, counter_data)
        self.hr_block.setValues(REG_PPR, [PPR])
        
        # Также обновляем Input Registers (для команды 04)
        self.ir_block.setValues(REG_ANGLE_RAD, angle_rad_data)
        self.ir_block.setValues(REG_ANGLE_DEG, angle_deg_data)
        self.ir_block.setValues(REG_COUNTER, counter_data)
        self.ir_block.setValues(REG_PPR, [PPR])

def run_modbus_server():
    """Запуск ModBus TCP сервера в отдельном потоке"""
    # Создание хранилища данных
    initial_values = [0] * 100
    initial_values[REG_PPR] = PPR
    
    # Создаем блоки данных для различных типов регистров
    hr_block = ModbusSequentialDataBlock(0, initial_values)  # Holding Registers
    ir_block = ModbusSequentialDataBlock(0, initial_values)  # Input Registers
    di_block = ModbusSequentialDataBlock(0, [0] * 100)       # Discrete Inputs
    co_block = ModbusSequentialDataBlock(0, [0] * 100)       # Coils
    
    # Создаем контекст устройства
    device_context = ModbusDeviceContext(
        di=di_block,    # Discrete Inputs
        co=co_block,    # Coils
        hr=hr_block,    # Holding Registers
        ir=ir_block     # Input Registers
    )
    
    # Создаем контекст сервера с указанием Device ID
    server_context = ModbusServerContext(devices={MODBUS_UNIT_ID: device_context}, single=False)
    
    data_store = ModbusDataStore(hr_block, ir_block)
    
    print(f"Запуск ModBus TCP сервера на порту {MODBUS_PORT}")
    print(f"Unit ID: {MODBUS_UNIT_ID}")
    print("Регистры:")
    print(f"  {REG_ANGLE_RAD}-{REG_ANGLE_RAD+1}: Угол в радианах (float32)")
    print(f"  {REG_ANGLE_DEG}-{REG_ANGLE_DEG+1}: Угол в градусах (float32)")
    print(f"  {REG_COUNTER}-{REG_COUNTER+1}: Счетчик импульсов (int32)")
    print(f"  {REG_PPR}: PPR энкодера (uint16)")
    
    # Запуск сервера в отдельном потоке
    def server_thread():
        StartTcpServer(server_context, address=("0.0.0.0", MODBUS_PORT))
    
    server_thread_obj = threading.Thread(target=server_thread, daemon=True)
    server_thread_obj.start()
    
    return data_store

def modbus_update_thread(data_store):
    """Поток для обновления ModBus регистров"""
    try:
        while True:
            data_store.update_registers()
            time.sleep(0.1)  # Обновление каждые 100мс
    except Exception as e:
        print(f"Ошибка в потоке ModBus: {e}")

def status_thread():
    """Поток для вывода статуса"""
    global counter, angle_rad, angle_deg
    try:
        while True:
            # Расчет углов
            angle_rad = (counter % PPR) * (2 * math.pi / PPR)
            angle_deg = math.degrees(angle_rad)
            
            print(f"Угол: {angle_rad:.3f} рад ({angle_deg:.1f}°), Счетчик: {counter}")
            time.sleep(1.0)  # Вывод каждую секунду
    except Exception as e:
        print(f"Ошибка в потоке статуса: {e}")

def main():
    """Основная функция (точно как rs.py + ModBus)"""
    global counter
    
    print("=== Raspberry Pi 3 Encoder + RS-485 + ModBus TCP (Sync) ===")
    
    # Устанавливаем высокий приоритет процессу
    try:
        os.nice(-20)  # Максимальный приоритет
        print("Установлен высокий приоритет процесса")
    except:
        print("Не удалось установить высокий приоритет процесса")
    
    pi_instance = pigpio.pi()
    
    # Инициализация энкодера
    encoder = EncoderReader(pigpio=pi_instance, a_pin=A_PIN, b_pin=B_PIN, z_pin=Z_PIN)
    try:
        encoder.start()
        print("Энкодер инициализирован")
    except Exception as e:
        print(f"Ошибка инициализации энкодера: {e}")
        return
    
    # Инициализация RS-485
    rs485 = RS485Transmitter(pi_instance, UART_DEVICE, UART_BAUDRATE, RS485_DE_PIN)
    try:
        rs485.start()
        print("RS-485 инициализирован")
    except Exception as e:
        print(f"Ошибка инициализации RS-485: {e}")
        encoder.stop()
        return
    
    # Запуск ModBus сервера
    try:
        data_store = run_modbus_server()
        print("ModBus сервер запущен")
    except Exception as e:
        print(f"Ошибка запуска ModBus сервера: {e}")
        encoder.stop()
        rs485.stop()
        return
    
    # Запуск потоков для ModBus и статуса
    modbus_thread = threading.Thread(target=modbus_update_thread, args=(data_store,), daemon=True)
    status_thread_obj = threading.Thread(target=status_thread, daemon=True)
    
    modbus_thread.start()
    status_thread_obj.start()
    
    print("Все потоки запущены")
    
    try:
        # Основной цикл (точно как в rs.py)
        while True:
            # Отправляем пакет напрямую по счетчику
            rs485.send_packet(counter)
            
    except KeyboardInterrupt:
        print("\nПолучен сигнал прерывания, остановка...")
    finally:
        rs485.stop()
        encoder.stop()
        print("Программа завершена")

if __name__ == "__main__":
    main()
