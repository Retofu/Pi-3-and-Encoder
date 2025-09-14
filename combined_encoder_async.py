#!/usr/bin/env python3
"""
Объединенный скрипт для работы с энкодером, RS-485 и ModBus TCP
Использует asyncio для работы в разных потоках
"""

import asyncio
import pigpio
import math
import time
import struct
import serial
import os
import threading
from typing import Optional
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

class SharedData:
    """Класс для безопасного обмена данными между потоками"""
    def __init__(self):
        self._lock = asyncio.Lock()
        self._counter = 0
        self._angle_rad = 0.0
        self._angle_deg = 0.0
        
    async def update_encoder_data(self, counter: int):
        async with self._lock:
            self._counter = counter
            self._angle_rad = counter * ANGLE_MULTIPLIER
            self._angle_deg = math.degrees(self._angle_rad)
    
    async def get_encoder_data(self):
        async with self._lock:
            return self._counter, self._angle_rad, self._angle_deg

class EncoderReader:
    def __init__(self, shared_data: SharedData):
        self._pi = None
        self._cb_a = None
        self._cb_z = None
        self._running = False
        self._shared_data = shared_data

    async def start(self):
        """Асинхронная инициализация энкодера"""
        if not self._pi.connected:
            raise RuntimeError("pigpio daemon is not running")

        # Настройка пинов
        self._pi.set_mode(A_PIN, pigpio.INPUT)
        self._pi.set_pull_up_down(A_PIN, pigpio.PUD_UP)
        self._pi.set_mode(B_PIN, pigpio.INPUT)
        self._pi.set_pull_up_down(B_PIN, pigpio.PUD_UP)
        self._pi.set_mode(Z_PIN, pigpio.INPUT)
        self._pi.set_pull_up_down(Z_PIN, pigpio.PUD_UP)

        # Фильтр дребезга
        self._pi.set_glitch_filter(A_PIN, 50)
        self._pi.set_glitch_filter(B_PIN, 50)
        self._pi.set_glitch_filter(Z_PIN, 50)

        # Обработчики прерываний
        self._cb_a = self._pi.callback(A_PIN, pigpio.EITHER_EDGE, self._handle_A)
        self._cb_z = self._pi.callback(Z_PIN, pigpio.RISING_EDGE, self._handle_Z)

        self._running = True
        print("Энкодер инициализирован")

    def stop(self):
        """Остановка энкодера"""
        self._running = False
        if self._cb_a:
            self._cb_a.cancel()
        if self._cb_z:
            self._cb_z.cancel()
        if self._pi:
            self._pi.stop()
        print("Энкодер остановлен")

    def _handle_A(self, gpio, level, tick):
        """Обработчик фазы A"""
        if level == pigpio.TIMEOUT or not self._running:
            return

        a = self._pi.read(A_PIN)
        b = self._pi.read(B_PIN)

        if level == 1:
            self._shared_data._counter += 1 if b == 0 else -1
        else:
            self._shared_data._counter += 1 if b == 1 else -1

    def _handle_Z(self, gpio, level, tick):
        """Обработчик индекса Z"""
        if level == 1 and self._running:
            self._shared_data._counter = 0

    async def update_shared_data(self):
        """Обновление общих данных"""
        await self._shared_data.update_encoder_data(self._shared_data._counter)

class RS485Transmitter:
    def __init__(self, shared_data: SharedData):
        self._device = UART_DEVICE
        self._baudrate = UART_BAUDRATE
        self._serial_port = None
        self._pi = None
        self._running = False
        self._shared_data = shared_data
        self._error_count = 0
        self._max_errors = 10
        
        # Создаем пакет
        self._packet = bytearray(120)
        self._packet[0] = 0x65
        self._packet[118] = 0x45
        self._packet[119] = 0xCF

    async def start(self):
        """Асинхронная инициализация RS-485"""
        try:
            if not self._pi.connected:
                raise RuntimeError("pigpio daemon is not running")

            self._pi.set_mode(RS485_DE_PIN, pigpio.OUTPUT)
            self._pi.write(RS485_DE_PIN, 0)

            # Закрываем порт если он был открыт
            if self._serial_port and self._serial_port.is_open:
                self._serial_port.close()

            self._serial_port = serial.Serial(
                port=self._device,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001,
                write_timeout=0.01  # Увеличиваем timeout для записи
            )

            # Очищаем буферы
            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()
            
            # Небольшая задержка для стабилизации
            await asyncio.sleep(0.01)

            self._running = True
            self._error_count = 0
            print("RS-485 инициализирован")

        except Exception as e:
            print(f"Ошибка инициализации RS-485: {e}")
            raise

    def stop(self):
        """Остановка RS-485"""
        self._running = False
        if self._pi:
            self._pi.write(RS485_DE_PIN, 0)
        if self._serial_port and self._serial_port.is_open:
            self._serial_port.close()
        print("RS-485 остановлен")

    async def _reconnect(self):
        """Переподключение RS-485"""
        try:
            print("Попытка переподключения RS-485...")
            self.stop()
            await asyncio.sleep(0.1)
            await self.start()
            return True
        except Exception as e:
            print(f"Ошибка переподключения RS-485: {e}")
            return False

    async def send_packet(self, counter_value: int):
        """Асинхронная отправка пакета с улучшенной обработкой ошибок"""
        if not self._running or not self._serial_port or not self._serial_port.is_open:
            return False

        try:
            # Вычисляем точный угол на основе счетчика
            angle = counter_value * ANGLE_MULTIPLIER
            self._packet[55:59] = struct.pack('<f', angle)

            # Контрольная сумма
            checksum = sum(self._packet[55:59], 0x65)
            self._packet[117] = 0xFF - (0xFF & checksum)

            # Включаем передачу перед отправкой
            self._pi.write(RS485_DE_PIN, 1)
            
            # Небольшая задержка для стабилизации DE
            await asyncio.sleep(0.0001)

            # Отправляем пакет
            bytes_written = self._serial_port.write(self._packet)
            
            # Ждем завершения передачи
            self._serial_port.flush()
            
            # Небольшая задержка после передачи
            await asyncio.sleep(0.0001)
            
            # Отключаем передачу
            self._pi.write(RS485_DE_PIN, 0)

            # Сбрасываем счетчик ошибок при успешной отправке
            if bytes_written == len(self._packet):
                self._error_count = 0
                return True
            else:
                raise Exception(f"Отправлено {bytes_written} из {len(self._packet)} байт")

        except Exception as e:
            self._error_count += 1
            print(f"Ошибка отправки RS-485 (попытка {self._error_count}): {e}")
            
            # Отключаем передачу в случае ошибки
            try:
                self._pi.write(RS485_DE_PIN, 0)
            except:
                pass
            
            # Если слишком много ошибок, пытаемся переподключиться
            if self._error_count >= self._max_errors:
                print(f"Слишком много ошибок RS-485 ({self._error_count}), переподключение...")
                if await self._reconnect():
                    self._error_count = 0
                else:
                    self._running = False
                    return False
            
            return False

class ModbusDataStore:
    """Класс для обновления данных в ModBus регистрах"""
    
    def __init__(self, hr_block, ir_block, shared_data: SharedData):
        self.hr_block = hr_block  # Holding Registers
        self.ir_block = ir_block  # Input Registers
        self._shared_data = shared_data
        
    async def update_registers(self):
        """Асинхронное обновление регистров данными энкодера"""
        counter, angle_rad, angle_deg = await self._shared_data.get_encoder_data()
        
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

def run_modbus_server(shared_data: SharedData):
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
    
    data_store = ModbusDataStore(hr_block, ir_block, shared_data)
    
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

async def encoder_task(shared_data: SharedData, pi_instance):
    """Задача для чтения энкодера"""
    encoder = EncoderReader(shared_data)
    encoder._pi = pi_instance
    
    try:
        await encoder.start()
        
        while True:
            await encoder.update_shared_data()
            await asyncio.sleep(0.001)  # Обновление каждую миллисекунду
            
    except Exception as e:
        print(f"Ошибка в задаче энкодера: {e}")
    finally:
        encoder.stop()

async def rs485_task(shared_data: SharedData, pi_instance):
    """Задача для передачи RS-485 с улучшенной обработкой ошибок"""
    rs485 = RS485Transmitter(shared_data)
    rs485._pi = pi_instance
    
    try:
        await rs485.start()
        
        consecutive_failures = 0
        max_consecutive_failures = 5
        
        while True:
            try:
                counter, _, _ = await shared_data.get_encoder_data()
                success = await rs485.send_packet(counter)
                
                if success:
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= max_consecutive_failures:
                        print(f"Слишком много последовательных ошибок RS-485 ({consecutive_failures})")
                        # Попытка переподключения
                        if await rs485._reconnect():
                            consecutive_failures = 0
                        else:
                            print("Не удалось переподключить RS-485, остановка задачи")
                            break
                
                await asyncio.sleep(0.003)  # Отправка каждые 3мс
                
            except Exception as e:
                print(f"Ошибка в цикле RS-485: {e}")
                consecutive_failures += 1
                await asyncio.sleep(0.01)  # Задержка при ошибке
                
    except Exception as e:
        print(f"Критическая ошибка в задаче RS-485: {e}")
    finally:
        rs485.stop()

async def modbus_task(shared_data: SharedData):
    """Задача для обновления ModBus регистров"""
    data_store = run_modbus_server(shared_data)
    
    try:
        while True:
            await data_store.update_registers()
            await asyncio.sleep(0.1)  # Обновление каждые 100мс
            
    except Exception as e:
        print(f"Ошибка в задаче ModBus: {e}")

async def status_task(shared_data: SharedData):
    """Задача для вывода статуса"""
    try:
        while True:
            counter, angle_rad, angle_deg = await shared_data.get_encoder_data()
            print(f"Угол: {angle_rad:.3f} рад ({angle_deg:.1f}°), Счетчик: {counter}")
            await asyncio.sleep(1.0)  # Вывод каждую секунду
            
    except Exception as e:
        print(f"Ошибка в задаче статуса: {e}")

async def main():
    """Основная асинхронная функция"""
    print("=== Raspberry Pi 3 Encoder + RS-485 + ModBus TCP (Async) ===")
    
    # Устанавливаем высокий приоритет процессу
    try:
        os.nice(-20)  # Максимальный приоритет
        print("Установлен высокий приоритет процесса")
    except:
        print("Не удалось установить высокий приоритет процесса")
    
    # Инициализация pigpio
    pi_instance = pigpio.pi()
    if not pi_instance.connected:
        print("Ошибка: pigpio daemon не запущен")
        return
    
    print("pigpio daemon подключен")
    
    # Создаем общие данные
    shared_data = SharedData()
    
    # Создаем задачи
    tasks = []
    
    try:
        # Создаем задачи с обработкой исключений
        encoder_task_obj = asyncio.create_task(encoder_task(shared_data, pi_instance))
        rs485_task_obj = asyncio.create_task(rs485_task(shared_data, pi_instance))
        modbus_task_obj = asyncio.create_task(modbus_task(shared_data))
        status_task_obj = asyncio.create_task(status_task(shared_data))
        
        tasks = [encoder_task_obj, rs485_task_obj, modbus_task_obj, status_task_obj]
        
        print("Все задачи запущены")
        
        # Ждем завершения задач
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        
        # Проверяем результаты
        for task in done:
            try:
                result = task.result()
            except Exception as e:
                print(f"Задача завершилась с ошибкой: {e}")
        
        # Отменяем оставшиеся задачи
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
                
    except KeyboardInterrupt:
        print("\nПолучен сигнал прерывания, остановка...")
    except Exception as e:
        print(f"Критическая ошибка в main: {e}")
    finally:
        # Отменяем все задачи
        for task in tasks:
            if not task.done():
                task.cancel()
        
        # Ждем завершения отмены
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        
        pi_instance.stop()
        print("Программа завершена")

if __name__ == "__main__":
    asyncio.run(main())
