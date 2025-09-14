#!/usr/bin/env python3
"""
Raspberry Pi 3 Encoder + Modbus Server + RS-485 Transmitter
Асинхронная версия с управлением GPIO25 для питания 27В
Плата работает как Modbus сервер, клиент подключается для настройки параметров
"""

import pigpio
import math
import time
import struct
import serial
import os
import asyncio
import logging
import threading
from typing import Optional, Dict, Any

from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext

class Endian:
    Big = 0
    Little = 1

class BinaryPayloadBuilder:
    def __init__(self, byteorder=0, wordorder=0):
        self.data = []
        self.byteorder = byteorder

    def add_32bit_float(self, value):
        import struct
        # Упаковываем float32 в big-endian (стандарт ModBus)
        packed = struct.pack('>f', value)
        # Разбиваем на два 16-битных регистра
        reg1 = int.from_bytes(packed[:2], 'big')
        reg2 = int.from_bytes(packed[2:], 'big')
        self.data.extend([reg1, reg2])

    def add_32bit_int(self, value):
        import struct
        # Упаковываем int32 в big-endian (стандарт ModBus)
        packed = struct.pack('>i', value)
        # Разбиваем на два 16-битных регистра
        reg1 = int.from_bytes(packed[:2], 'big')
        reg2 = int.from_bytes(packed[2:], 'big')
        self.data.extend([reg1, reg2])

    def to_registers(self):
        return self.data

class BinaryPayloadDecoder:
    def __init__(self, payload, byteorder=0, wordorder=0):
        self.payload = payload
        self.byteorder = byteorder
        self.wordorder = wordorder
        self.pointer = 0

    def decode_32bit_float(self):
        """Декодирование float32 из двух 16-битных регистров"""
        if self.pointer + 1 >= len(self.payload):
            raise ValueError("Недостаточно данных для декодирования float32")
        
        reg1 = self.payload[self.pointer]
        reg2 = self.payload[self.pointer + 1]
        self.pointer += 2
        
        # Упаковываем в big-endian (стандарт ModBus)
        packed = struct.pack('>HH', reg1, reg2)
        return struct.unpack('>f', packed)[0]

    def decode_32bit_int(self):
        """Декодирование int32 из двух 16-битных регистров"""
        if self.pointer + 1 >= len(self.payload):
            raise ValueError("Недостаточно данных для декодирования int32")
        
        reg1 = self.payload[self.pointer]
        reg2 = self.payload[self.pointer + 1]
        self.pointer += 2
        
        # Упаковываем в big-endian (стандарт ModBus)
        packed = struct.pack('>HH', reg1, reg2)
        return struct.unpack('>i', packed)[0]

    def decode_16bit_uint(self):
        """Декодирование uint16 из одного регистра"""
        if self.pointer >= len(self.payload):
            raise ValueError("Недостаточно данных для декодирования uint16")
        
        value = self.payload[self.pointer]
        self.pointer += 1
        return value

    def reset(self):
        """Сброс указателя в начало"""
        self.pointer = 0

# Настройка пинов энкодера
A_PIN = 17
B_PIN = 22
Z_PIN = 27

# Настройка пинов RS-485 и управления питанием
RS485_DE_PIN = 24
POWER_27V_PIN = 25  # GPIO25 для управления питанием 27В

# Настройки UART
UART_DEVICE = '/dev/serial0'
UART_BAUDRATE = 507000

# Настройки Modbus сервера
MODBUS_SERVER_PORT = 502
MODBUS_UNIT_ID = 255

# Адреса регистров согласно таблице
# Feedback registers (1000-1019)
FBK_DELAY_BEFORE = 1000
FBK_DELAY_COUNT = 1001
FBK_POS_COUNT = 1002
FBK_POS_COUNT_MAX = 1003
FBK_POS = 1004
FBK_PULSE_LENGTH = 1005
FBK_PULSE_COUNT = 1006
FBK_FRONT_TYPE = 1007
FBK_PULSE_ON = 1008
FBK_POWER_27_V = 1009
FBK_POS_SET = 1010
FBK_RESERVE_11 = 1011
FBK_ANGLE_ROLL = 1012  # FLOAT (2 registers)
FBK_ANGLE_ADJ = 1014   # FLOAT (2 registers)
FBK_RESERVE_16 = 1016
FBK_RESERVE_17 = 1017
FBK_RESERVE_18 = 1018
FBK_RESERVE_19 = 1019

# Setpoint registers (2000-2013)
SP_DELAY_BEFORE = 2000
SP_RESERVE_1 = 2001
SP_POS_COUNT = 2002
SP_POS_COUNT_MAX = 2003
SP_RESERVE_4 = 2004
SP_PULSE_LENGTH = 2005
SP_RESERVE_6 = 2006
SP_FRONT_TYPE = 2007
SP_PULSE_ON = 2008
SP_POWER_27_V = 2009
SP_POS_SET = 2010
SP_STATUS = 2011
SP_ANGLE_OFFSET = 2012  # FLOAT (2 registers)

# Константы для RS-485 пакета
PACKET_SIZE = 120
ANGLE_BYTE_START = 55  # Байты 56-59 (индекс 55-58)
STATUS_BYTE = 116      # Байт 117 (индекс 116)

# Глобальные переменные
counter = 0
power_27v_enabled = False  # Состояние питания 27В

# Настройка логирования
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ModbusDataStore:
    """Класс для управления данными Modbus сервера"""
    
    def __init__(self, hr_block, ir_block):
        self.hr_block = hr_block  # Holding Registers
        self.ir_block = ir_block  # Input Registers
        self.variables = {}  # Кэш переменных
        
    def update_encoder_data(self, counter: int, angle_rad: float, angle_deg: float):
        """Обновление данных энкодера в регистрах обратной связи"""
        try:
            # Обновляем счетчик импульсов
            self.hr_block.setValues(FBK_POS_COUNT, [counter])
            self.ir_block.setValues(FBK_POS_COUNT, [counter])
            
            # Обновляем угол в градусах
            self.hr_block.setValues(FBK_POS, [int(angle_deg)])
            self.ir_block.setValues(FBK_POS, [int(angle_deg)])
            
            # Обновляем угол в радианах (FLOAT, 2 регистра)
            builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
            builder.add_32bit_float(angle_rad)
            angle_rad_data = builder.to_registers()
            
            self.hr_block.setValues(FBK_ANGLE_ROLL, angle_rad_data)
            self.ir_block.setValues(FBK_ANGLE_ROLL, angle_rad_data)
            
        except Exception as e:
            logger.error(f"Ошибка обновления данных энкодера: {e}")
    
    def get_setpoint_variable(self, register: int) -> Any:
        """Получение переменной задания из регистра"""
        try:
            if register == SP_ANGLE_OFFSET:
                # FLOAT регистр (2 слова)
                reg1 = self.hr_block.getValues(SP_ANGLE_OFFSET, 1)[0]
                reg2 = self.hr_block.getValues(SP_ANGLE_OFFSET + 1, 1)[0]
                decoder = BinaryPayloadDecoder([reg1, reg2], byteorder=Endian.Big, wordorder=Endian.Big)
                return decoder.decode_32bit_float()
            else:
                # UINT регистр
                return self.hr_block.getValues(register, 1)[0]
        except Exception as e:
            logger.error(f"Ошибка чтения регистра {register}: {e}")
            return 0
    
    def get_power_27v_command(self) -> bool:
        """Получение команды питания 27В"""
        return self.get_setpoint_variable(SP_POWER_27_V) == 1
    
    def get_angle_offset(self) -> float:
        """Получение смещения угла"""
        return self.get_setpoint_variable(SP_ANGLE_OFFSET)
    
    def get_status_word(self) -> int:
        """Получение статусного слова"""
        return self.get_setpoint_variable(SP_STATUS)
    
    def get_pos_count_max(self) -> int:
        """Получение максимального количества импульсов на оборот"""
        return self.get_setpoint_variable(SP_POS_COUNT_MAX)

class PowerController:
    """Класс для управления питанием 27В через GPIO25"""
    
    def __init__(self, pi_instance, pin: int):
        self.pi = pi_instance
        self.pin = pin
        self.enabled = False
        
    def setup(self):
        """Настройка пина для управления питанием"""
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.write(self.pin, 0)  # Изначально выключено
        logger.info(f"Настроен пин {self.pin} для управления питанием 27В")
    
    def set_power(self, enable: bool):
        """Включение/выключение питания 27В"""
        self.pi.write(self.pin, 1 if enable else 0)
        self.enabled = enable
        logger.info(f"Питание 27В: {'включено' if enable else 'выключено'}")
    
    def cleanup(self):
        """Очистка ресурсов"""
        self.pi.write(self.pin, 0)
        self.enabled = False

class EncoderReader:
    """Класс для чтения энкодера"""
    
    def __init__(self, pi_instance, a_pin: int, b_pin: int, z_pin: int):
        self.pi = pi_instance
        self.a_pin = a_pin
        self.b_pin = b_pin
        self.z_pin = z_pin
        self.cb_a = None
        self.cb_z = None
        self.running = False
        
    def start(self):
        """Запуск чтения энкодера"""
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running")
        
        # Настройка пинов
        self.pi.set_mode(self.a_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.a_pin, pigpio.PUD_UP)
        self.pi.set_mode(self.b_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.b_pin, pigpio.PUD_UP)
        self.pi.set_mode(self.z_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.z_pin, pigpio.PUD_UP)
        
        # Фильтр дребезга
        self.pi.set_glitch_filter(self.a_pin, 50)
        self.pi.set_glitch_filter(self.b_pin, 50)
        self.pi.set_glitch_filter(self.z_pin, 50)
        
        # Обработчики прерываний
        self.cb_a = self.pi.callback(self.a_pin, pigpio.EITHER_EDGE, self._handle_A)
        self.cb_z = self.pi.callback(self.z_pin, pigpio.RISING_EDGE, self._handle_Z)
        
        self.running = True
        logger.info("Энкодер запущен")
        
    def stop(self):
        """Остановка чтения энкодера"""
        self.running = False
        if self.cb_a:
            self.cb_a.cancel()
        if self.cb_z:
            self.cb_z.cancel()
        logger.info("Энкодер остановлен")
        
    def _handle_A(self, gpio, level, tick):
        """Обработчик фазы A"""
        global counter
        if level == pigpio.TIMEOUT or not self.running:
            return
        
        try:
            a = self.pi.read(self.a_pin)
            b = self.pi.read(self.b_pin)
            
            if level == 1:
                counter += 1 if b == 0 else -1
            else:
                counter += 1 if b == 1 else -1
        except:
            pass
            
    def _handle_Z(self, gpio, level, tick):
        """Обработчик индекса Z"""
        global counter
        if level == 1 and self.running:
            counter = 0

class RS485Transmitter:
    """Класс для передачи данных по RS-485"""
    
    def __init__(self, pi_instance, device: str, baudrate: int, rs485_de_pin: int):
        self.pi = pi_instance
        self.device = device
        self.baudrate = baudrate
        self.rs485_de_pin = rs485_de_pin
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        
        # Создаем пакет согласно спецификации (120 байт)
        self.packet = bytearray(PACKET_SIZE)
        # Байты 1-55: нули (уже инициализированы)
        # Байты 56-59: угол (будет заполняться)
        # Байты 60-80: нули (уже инициализированы)
        # Байты 81-82: статусное слово (будет заполняться)
        # Байты 83-116: нули (уже инициализированы)
        # Байты 117-120: контрольная сумма и заголовок (будет заполняться)
        
    def start(self):
        """Запуск RS-485 передатчика"""
        try:
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon is not running")
            
            # Настройка пина DE для RS-485
            self.pi.set_mode(self.rs485_de_pin, pigpio.OUTPUT)
            self.pi.write(self.rs485_de_pin, 0)
            
            # Настройка UART
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
            logger.info(f"RS-485 передатчик запущен на {self.device}")
            
        except Exception as e:
            logger.error(f"Ошибка запуска RS-485: {e}")
            raise
    
    def stop(self):
        """Остановка RS-485 передатчика"""
        self.running = False
        if self.pi:
            self.pi.write(self.rs485_de_pin, 0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        logger.info("RS-485 передатчик остановлен")
    
    def calculate_angle_roll(self, angle_encoder: float, offset_angle_roll: float) -> float:
        """
        Расчет угла крена по формуле:
        Angle_roll = (Mod360(Offset_angle_roll + angle_encoder)) * π / 180
        """
        # Mod360 - приведение к диапазону 0-360 градусов
        angle_deg = (offset_angle_roll + angle_encoder) % 360.0
        # Преобразование в радианы
        angle_rad = angle_deg * math.pi / 180.0
        return angle_rad
    
    def send_packet(self, angle_encoder: float, status_word: int = 0, angle_offset: float = 0.0):
        """Отправка пакета по RS-485"""
        if not self.running or not self.serial_port:
            return False
        
        try:
            # Вычисляем угол крена
            angle_roll = self.calculate_angle_roll(angle_encoder, angle_offset)
            
            # Заполняем байты 56-59 (индекс 55-58) - угол в радианах (float, little-endian)
            angle_bytes = struct.pack('<f', angle_roll)
            self.packet[ANGLE_BYTE_START:ANGLE_BYTE_START+4] = angle_bytes
            
            # Заполняем байты 81-82 (индекс 80-81) - статусное слово
            status_bytes = struct.pack('<H', status_word)
            self.packet[80:82] = status_bytes
            
            # Включаем передачу
            self.pi.write(self.rs485_de_pin, 1)
            
            # Отправляем пакет
            self.serial_port.write(self.packet)
            
            # Выключаем передачу
            self.pi.write(self.rs485_de_pin, 0)
            
            return True
            
        except Exception as e:
            logger.error(f"Ошибка отправки пакета RS-485: {e}")
            # В случае ошибки выключаем передачу
            try:
                self.pi.write(self.rs485_de_pin, 0)
            except:
                pass
            return False

def run_modbus_server(data_store: ModbusDataStore) -> threading.Thread:
    """Запуск Modbus TCP сервера"""
    def server_thread():
        try:
            # Создаем контекст устройства
            device_context = ModbusDeviceContext(
                di=ModbusSequentialDataBlock(0, [0] * 100),    # Discrete Inputs
                co=ModbusSequentialDataBlock(0, [0] * 100),    # Coils
                hr=data_store.hr_block,                        # Holding Registers
                ir=data_store.ir_block                         # Input Registers
            )
            
            # Создаем контекст сервера
            server_context = ModbusServerContext(devices={MODBUS_UNIT_ID: device_context}, single=False)
            
            logger.info(f"Запуск Modbus TCP сервера на порту {MODBUS_SERVER_PORT}")
            logger.info(f"Unit ID: {MODBUS_UNIT_ID}")
            
            # Запускаем сервер
            StartTcpServer(server_context, address=("0.0.0.0", MODBUS_SERVER_PORT))
            
        except Exception as e:
            logger.error(f"Ошибка запуска Modbus сервера: {e}")
    
    server_thread_obj = threading.Thread(target=server_thread, daemon=True)
    server_thread_obj.start()
    return server_thread_obj

async def power_control_task(power_controller: PowerController, data_store: ModbusDataStore):
    """Задача управления питанием 27В"""
    global power_27v_enabled
    
    while True:
        try:
            # Проверяем команду питания 27В из Modbus регистров
            power_command = data_store.get_power_27v_command()
            
            if power_command != power_27v_enabled:
                power_controller.set_power(power_command)
                power_27v_enabled = power_command
                logger.info(f"Питание 27В: {'включено' if power_command else 'выключено'}")
            
            await asyncio.sleep(0.1)  # Проверяем каждые 100мс
            
        except Exception as e:
            logger.error(f"Ошибка в задаче управления питанием: {e}")
            await asyncio.sleep(1)

async def rs485_transmission_task(rs485_transmitter: RS485Transmitter, data_store: ModbusDataStore):
    """Задача передачи данных по RS-485"""
    global counter
    
    while True:
        try:
            # Получаем параметры из Modbus регистров
            ppr = data_store.get_pos_count_max()
            if ppr == 0:
                ppr = 360  # Значение по умолчанию
            
            angle_offset = data_store.get_angle_offset()
            status_word = data_store.get_status_word()
            
            # Вычисляем угол энкодера в градусах
            angle_encoder_deg = (counter % ppr) * (360.0 / ppr)
            
            # Отправляем пакет
            rs485_transmitter.send_packet(angle_encoder_deg, status_word, angle_offset)
            
            await asyncio.sleep(0.003)  # Цикл 3мс (как в оригинале)
            
        except Exception as e:
            logger.error(f"Ошибка в задаче RS-485: {e}")
            await asyncio.sleep(0.1)

async def encoder_update_task(encoder: EncoderReader, data_store: ModbusDataStore):
    """Задача обновления данных энкодера в Modbus регистрах"""
    global counter
    
    while True:
        try:
            # Вычисляем углы
            ppr = data_store.get_pos_count_max()
            if ppr == 0:
                ppr = 360  # Значение по умолчанию
            
            angle_rad = (counter % ppr) * (2 * math.pi / ppr)
            angle_deg = math.degrees(angle_rad)
            
            # Обновляем данные в Modbus регистрах
            data_store.update_encoder_data(counter, angle_rad, angle_deg)
            
            await asyncio.sleep(0.1)  # Обновляем каждые 100мс
            
        except Exception as e:
            logger.error(f"Ошибка в задаче обновления энкодера: {e}")
            await asyncio.sleep(0.1)

async def main():
    """Основная асинхронная функция"""
    global counter
    
    logger.info("=== Raspberry Pi 3 Encoder + Modbus Server + RS-485 ===")
    
    # Устанавливаем высокий приоритет процессу
    try:
        os.nice(-20)  # Максимальный приоритет
    except:
        pass
    
    # Инициализация pigpio
    pi_instance = pigpio.pi()
    if not pi_instance.connected:
        logger.error("pigpio daemon не запущен")
        return
    
    try:
        # Создаем блоки данных для Modbus
        initial_values = [0] * 3000  # Достаточно для всех регистров
        hr_block = ModbusSequentialDataBlock(0, initial_values)  # Holding Registers
        ir_block = ModbusSequentialDataBlock(0, initial_values)  # Input Registers
        
        # Создаем хранилище данных
        data_store = ModbusDataStore(hr_block, ir_block)
        
        # Инициализация энкодера
        encoder = EncoderReader(pi_instance, A_PIN, B_PIN, Z_PIN)
        encoder.start()
        
        # Инициализация контроллера питания
        power_controller = PowerController(pi_instance, POWER_27V_PIN)
        power_controller.setup()
        
        # Инициализация RS-485 передатчика
        rs485_transmitter = RS485Transmitter(pi_instance, UART_DEVICE, UART_BAUDRATE, RS485_DE_PIN)
        rs485_transmitter.start()
        
        # Запуск Modbus сервера
        server_thread = run_modbus_server(data_store)
        
        logger.info("Все системы инициализированы. Начинаем работу...")
        logger.info("Modbus сервер запущен. Клиент может подключаться для настройки параметров.")
        
        # Запускаем задачи параллельно
        tasks = [
            asyncio.create_task(power_control_task(power_controller, data_store)),
            asyncio.create_task(rs485_transmission_task(rs485_transmitter, data_store)),
            asyncio.create_task(encoder_update_task(encoder, data_store))
        ]
        
        # Ждем завершения задач
        await asyncio.gather(*tasks)
        
    except KeyboardInterrupt:
        logger.info("Получен сигнал остановки...")
    except Exception as e:
        logger.error(f"Критическая ошибка: {e}")
    finally:
        # Очистка ресурсов
        try:
            power_controller.cleanup()
            rs485_transmitter.stop()
            encoder.stop()
            pi_instance.stop()
        except:
            pass
        logger.info("Программа завершена")

if __name__ == "__main__":
    asyncio.run(main())
