#!/usr/bin/env python3
"""
Raspberry Pi 3 Encoder + Modbus Server + RS-485 Transmitter
Асинхронная версия с управлением GPIO25 для питания 27В
Симплексный режим RS-485 (постоянная передача)
ИСПРАВЛЕННАЯ ВЕРСИЯ - устранены проблемы с синхронизацией и таймингом
"""

import asyncio
import logging
import math
import os
import struct
import threading
import time
import ctypes
from typing import Optional, Dict, Any

import pigpio
import serial
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext

class Endian:
    """Константы для порядка байтов"""
    Big = 0
    Little = 1

# Настройки пинов GPIO
class GpioPins:
    """Константы для GPIO пинов"""
    ENCODER_A = 17
    ENCODER_B = 22
    ENCODER_Z = 27
    RS485_DE = 24
    POWER_27V = 25

# Настройки UART
class UartConfig:
    """Конфигурация UART"""
    DEVICE = '/dev/serial0'
    BAUDRATE = 507000

# Настройки Modbus сервера
class ModbusConfig:
    """Конфигурация Modbus сервера"""
    SERVER_PORT = 2502
    UNIT_ID = 1

# Константы для RS-485 пакета
class Rs485Config:
    """Конфигурация RS-485 пакета"""
    PACKET_SIZE = 120  # Размер пакета: 120 байт (индексы 0-119)
    ANGLE_BYTE_START = 55  # Байты 56-59 (индекс 55-58)
    STATUS_BYTE_START = 80 # Байты 81-82 (индекс 80-81)
    CHECKSUM_BYTE = 117    # Байт 118 (индекс 117)
    
    # Постоянные байты пакета (исправлены индексы)
    HEADER_BYTE_0 = 0x65   # Байт 1 (индекс 0)
    HEADER_BYTE_118 = 0x45 # Байт 119 (индекс 118)
    HEADER_BYTE_119 = 0xCF # Байт 120 (индекс 119)

# Адреса Modbus регистров
class ModbusRegisters:
    """Адреса Modbus регистров"""
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

class BinaryPayloadBuilder:
    def __init__(self, byteorder=0, wordorder=0):
        self.data = []
        self.byteorder = byteorder

    def add_32bit_float(self, value):
        import struct
        packed = struct.pack('>f', value)
        reg1 = int.from_bytes(packed[:2], 'big')
        reg2 = int.from_bytes(packed[2:], 'big')
        self.data.extend([reg1, reg2])

    def add_32bit_int(self, value):
        import struct
        packed = struct.pack('>i', value)
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
        
        packed = struct.pack('>HH', reg1, reg2)
        return struct.unpack('>f', packed)[0]

    def decode_32bit_int(self):
        """Декодирование int32 из двух 16-битных регистров"""
        if self.pointer + 1 >= len(self.payload):
            raise ValueError("Недостаточно данных для декодирования int32")
        
        reg1 = self.payload[self.pointer]
        reg2 = self.payload[self.pointer + 1]
        self.pointer += 2
        
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

class DataChangeTracker:
    """Отслеживание изменений данных Modbus"""
    
    def __init__(self):
        self.last_ppr = 360
        self.last_angle_offset = 0.0
        self.last_status_word = 0
        self.dirty = True
        
    def update(self, ppr: int, angle_offset: float, status_word: int) -> bool:
        """Обновление данных и возврат флага изменения"""
        changed = False
        
        if ppr != self.last_ppr:
            self.last_ppr = ppr
            changed = True
            
        if abs(angle_offset - self.last_angle_offset) > 0.0001:
            self.last_angle_offset = angle_offset
            changed = True
            
        if status_word != self.last_status_word:
            self.last_status_word = status_word
            changed = True
            
        if changed:
            self.dirty = True
            
        return changed
    
    def mark_clean(self):
        """Отметить данные как обработанные"""
        self.dirty = False
    
    def is_dirty(self) -> bool:
        """Проверить, изменились ли данные"""
        return self.dirty

# ============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНОЙ ЗАДЕРЖКИ
# ============================================================================

def usleep(microseconds):
    """Точная задержка в микросекундах"""
    if os.name == 'nt':  # Windows
        ctypes.windll.kernel32.Sleep(int(microseconds / 1000))
    else:  # Linux/Unix
        ctypes.CDLL('libc.so.6').usleep(microseconds)

# ============================================================================
# ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ И НАСТРОЙКИ
# ============================================================================

# Глобальные переменные с синхронизацией
counter = 0
counter_lock = threading.Lock()  # Блокировка для безопасного доступа к counter
power_27v_enabled = False

# Настройка логирования
logging.basicConfig(
    level=logging.INFO, 
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ============================================================================
# КЛАССЫ ДЛЯ РАБОТЫ С MODBUS
# ============================================================================

class ModbusDataStore:
    """Класс для управления данными Modbus сервера"""
    
    def __init__(self, hr_block, ir_block):
        self.hr_block = hr_block
        self.ir_block = ir_block
        self.variables = {}
        
    def update_encoder_data(self, counter: int, angle_rad: float, angle_deg: float):
        """Обновление данных энкодера в регистрах обратной связи"""
        try:
            self.hr_block.setValues(ModbusRegisters.FBK_POS_COUNT, [counter])
            self.ir_block.setValues(ModbusRegisters.FBK_POS_COUNT, [counter])
            
            self.hr_block.setValues(ModbusRegisters.FBK_POS, [int(angle_deg)])
            self.ir_block.setValues(ModbusRegisters.FBK_POS, [int(angle_deg)])
            
            builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
            builder.add_32bit_float(angle_rad)
            angle_rad_data = builder.to_registers()
        
            self.hr_block.setValues(ModbusRegisters.FBK_ANGLE_ROLL, angle_rad_data)
            self.ir_block.setValues(ModbusRegisters.FBK_ANGLE_ROLL, angle_rad_data)
            
        except Exception as e:
            logger.error(f"Ошибка обновления данных энкодера: {e}")
    
    def get_setpoint_variable(self, register: int) -> Any:
        """Получение переменной задания из регистра"""
        try:
            if register == ModbusRegisters.SP_ANGLE_OFFSET:
                reg1 = self.hr_block.getValues(ModbusRegisters.SP_ANGLE_OFFSET, 1)[0]
                reg2 = self.hr_block.getValues(ModbusRegisters.SP_ANGLE_OFFSET + 1, 1)[0]
                decoder = BinaryPayloadDecoder([reg1, reg2], byteorder=Endian.Big, wordorder=Endian.Big)
                return decoder.decode_32bit_float()
            else:
                return self.hr_block.getValues(register, 1)[0]
        except Exception as e:
            logger.error(f"Ошибка чтения регистра {register}: {e}")
            return 0
    
    def get_power_27v_command(self) -> bool:
        """Получение команды питания 27В"""
        return self.get_setpoint_variable(ModbusRegisters.SP_POWER_27_V) == 1
    
    def get_angle_offset(self) -> float:
        """Получение смещения угла"""
        return self.get_setpoint_variable(ModbusRegisters.SP_ANGLE_OFFSET)
    
    def get_status_word(self) -> int:
        """Получение статусного слова"""
        return self.get_setpoint_variable(ModbusRegisters.SP_STATUS)
    
    def get_pos_count_max(self) -> int:
        """Получение максимального количества импульсов на оборот"""
        return self.get_setpoint_variable(ModbusRegisters.SP_POS_COUNT_MAX)

# ============================================================================
# КЛАССЫ ДЛЯ УПРАВЛЕНИЯ ОБОРУДОВАНИЕМ
# ============================================================================

class PowerController:
    """Класс для управления питанием 27В через GPIO25"""
    
    def __init__(self, pi_instance, pin: int):
        self.pi = pi_instance
        self.pin = pin
        self.enabled = False
        
    def setup(self):
        """Настройка пина для управления питанием"""
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.write(self.pin, 0)
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

        self.pi.set_mode(self.a_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.a_pin, pigpio.PUD_UP)
        self.pi.set_mode(self.b_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.b_pin, pigpio.PUD_UP)
        self.pi.set_mode(self.z_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.z_pin, pigpio.PUD_UP)
        
        self.pi.set_glitch_filter(self.a_pin, 50)
        self.pi.set_glitch_filter(self.b_pin, 50)
        self.pi.set_glitch_filter(self.z_pin, 50)

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

            # Безопасное изменение counter с блокировкой
            with counter_lock:
                if level == 1:
                    counter += 1 if b == 0 else -1
                else:
                    counter += 1 if b == 1 else -1
        except Exception as e:
            logger.error(f"Ошибка в обработчике фазы A: {e}")

    def _handle_Z(self, gpio, level, tick):
        """Обработчик индекса Z"""
        global counter
        if level == 1 and self.running:
            # Безопасное сброса counter с блокировкой
            with counter_lock:
                counter = 0

class RS485Transmitter:
    """Класс для передачи данных по RS-485 в симплексном режиме"""
    
    def __init__(self, pi_instance, device: str, baudrate: int, rs485_de_pin: int):
        self.pi = pi_instance
        self.device = device
        self.baudrate = baudrate
        self.rs485_de_pin = rs485_de_pin
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        
        self.packet = bytearray(Rs485Config.PACKET_SIZE)
        self.packet[0] = Rs485Config.HEADER_BYTE_0
        self.packet[118] = Rs485Config.HEADER_BYTE_118
        self.packet[119] = Rs485Config.HEADER_BYTE_119
        
        self.change_tracker = DataChangeTracker()
        self.need_update = True

    def start(self):
        """Запуск RS-485 передатчика в симплексном режиме"""
        try:
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon is not running")

            # ВКЛЮЧАЕМ ПЕРЕДАЧУ НАПОСТОЯННО
            self.pi.set_mode(self.rs485_de_pin, pigpio.OUTPUT)
            self.pi.write(self.rs485_de_pin, 1)
            
            self.serial_port = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,  # Увеличиваем timeout для стабильности
                write_timeout=0.01,  # Увеличиваем write_timeout для стабильности
                inter_byte_timeout=0.01,  # Увеличиваем inter_byte_timeout
                xonxoff=False,  # Отключаем XON/XOFF flow control
                rtscts=False,   # Отключаем RTS/CTS flow control
                dsrdtr=False    # Отключаем DSR/DTR flow control
            )

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # Дополнительная очистка буферов
            self.serial_port.flush()
            time.sleep(0.01)  # Даем время на очистку
            
            self.running = True
            logger.info(f"RS-485 передатчик запущен в симплексном режиме на {self.device}")

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
        """Расчет угла крена"""
        angle_deg = (offset_angle_roll + angle_encoder) % 360.0
        angle_rad = angle_deg * math.pi / 180.0
        return angle_rad
    
    def calculate_checksum(self) -> int:
        """Вычисление контрольной суммы"""
        byte_sum = sum(self.packet[0:117])
        checksum = 0xFF - (0xFF & byte_sum)
        return checksum
    
    def update_parameters(self, ppr: int, angle_offset: float, status_word: int):
        """Обновление параметров и проверка изменений"""
        return self.change_tracker.update(ppr, angle_offset, status_word)
    
    def prepare_packet(self, angle_encoder_deg: float) -> bool:
        """Подготовка пакета - всегда обновляем угол энкодера"""
        try:
            # Проверяем валидность данных
            if not isinstance(angle_encoder_deg, (int, float)):
                logger.error(f"Некорректный угол энкодера: {angle_encoder_deg}")
                return False
                
            # Ограничиваем угол
            angle_encoder_deg = angle_encoder_deg % 360.0
            
            # В симплексном режиме всегда обновляем угол энкодера
            # Вычисляем угол крена
            angle_roll = self.calculate_angle_roll(
                angle_encoder_deg, 
                self.change_tracker.last_angle_offset
            )
            
            # Заполняем байты 56-59 - угол в радианах
            angle_bytes = struct.pack('<f', angle_roll)
            self.packet[Rs485Config.ANGLE_BYTE_START:Rs485Config.ANGLE_BYTE_START+4] = angle_bytes
            
            # Обновляем статусное слово только если параметры изменились
            if self.change_tracker.is_dirty():
                # Заполняем байты 81-82 - статусное слово
                status_bytes = struct.pack('<H', self.change_tracker.last_status_word)
                self.packet[Rs485Config.STATUS_BYTE_START:Rs485Config.STATUS_BYTE_START+2] = status_bytes
                self.change_tracker.mark_clean()
            
            # Вычисляем и устанавливаем контрольную сумму
            checksum = self.calculate_checksum()
            self.packet[Rs485Config.CHECKSUM_BYTE] = checksum
            
            return True
            
        except Exception as e:
            logger.error(f"Ошибка подготовки пакета: {e}")
            return False
    
    def send_packet(self):
        """Отправка подготовленного пакета с ожиданием завершения передачи"""
        if not self.running or not self.serial_port:
            return False

        try:
            # Проверяем размер пакета
            if len(self.packet) != Rs485Config.PACKET_SIZE:
                logger.error(f"Неправильный размер пакета: {len(self.packet)} байт, ожидается {Rs485Config.PACKET_SIZE}")
                return False
                
            # Отправляем данные
            bytes_written = self.serial_port.write(self.packet)
            if bytes_written != len(self.packet):
                logger.error(f"Записано {bytes_written} из {len(self.packet)} байт")
                return False
                
            # Ждем завершения передачи (2.4мс для 120 байт при 507000 бод)
            transmission_time = 0.0024  # 2.4мс
            start_time = time.perf_counter()
            
            # Ждем пока данные передадутся
            while time.perf_counter() - start_time < transmission_time:
                pass
                
            # Принудительно ждем завершения передачи
            self.serial_port.flush()
            
            return True
            
        except Exception as e:
            logger.error(f"Ошибка отправки пакета RS-485: {e}")
            return False

# ============================================================================
# ФУНКЦИИ MODBUS СЕРВЕРА
# ============================================================================

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
            server_context = ModbusServerContext(devices={ModbusConfig.UNIT_ID: device_context}, single=False)
            
            logger.info(f"Запуск Modbus TCP сервера на порту {ModbusConfig.SERVER_PORT}")
            logger.info(f"Unit ID: {ModbusConfig.UNIT_ID}")
            
            # Запускаем сервер
            StartTcpServer(server_context, address=("0.0.0.0", ModbusConfig.SERVER_PORT))
            
        except Exception as e:
            logger.error(f"Ошибка запуска Modbus сервера: {e}")
    
    server_thread_obj = threading.Thread(target=server_thread, daemon=True)
    server_thread_obj.start()
    return server_thread_obj

# ============================================================================
# АСИНХРОННЫЕ ЗАДАЧИ
# ============================================================================

async def power_control_task(power_controller: PowerController, data_store: ModbusDataStore):
    """Задача управления питанием 27В"""
    last_power_state = False
    
    while True:
        try:
            # Проверяем команду питания 27В из Modbus регистров
            power_command = data_store.get_power_27v_command()
            
            if power_command != last_power_state:
                power_controller.set_power(power_command)
                last_power_state = power_command
                logger.info(f"Питание 27В: {'включено' if power_command else 'выключено'}")
            
            await asyncio.sleep(0.1)  # Проверяем каждые 100мс
            
        except Exception as e:
            logger.error(f"Ошибка в задаче управления питанием: {e}")
            await asyncio.sleep(1)

async def parameter_update_task(data_store: ModbusDataStore, rs485_transmitter: RS485Transmitter):
    """Задача обновления параметров из Modbus"""
    while True:
        try:
            # Читаем параметры из Modbus
            ppr = data_store.get_pos_count_max()
            angle_offset = data_store.get_angle_offset()
            status_word = data_store.get_status_word()
            
            # Обновляем параметры в передатчике
            rs485_transmitter.update_parameters(ppr, angle_offset, status_word)
            
            await asyncio.sleep(0.1)  # Проверяем изменения каждые 100мс
            
        except Exception as e:
            logger.error(f"Ошибка в задаче обновления параметров: {e}")
            await asyncio.sleep(1)

def rs485_transmission_task(rs485_transmitter: RS485Transmitter):
    """Задача передачи данных по RS-485 в симплексном режиме"""
    global counter
    
    packet_count = 0
    error_count = 0
    last_log_time = time.time()
    last_packet_time = time.perf_counter()
    
    while True:
        try:
            # Начало цикла
            cycle_start = time.perf_counter()
            
            # Получаем текущий PPR из трекера
            ppr = rs485_transmitter.change_tracker.last_ppr
            if ppr == 0:
                ppr = 360
                
            # Безопасное чтение counter с блокировкой
            with counter_lock:
                current_counter = counter
                
            # Вычисляем угол энкодера в каждом цикле (для симплексного режима)
            angle_encoder_deg = (current_counter % ppr) * (360.0 / ppr)
            
            # Подготавливаем пакет (всегда обновляем угол)
            if rs485_transmitter.prepare_packet(angle_encoder_deg):
                # Отправляем пакет ВСЕГДА (симплексный режим)
                if rs485_transmitter.send_packet():
                    packet_count += 1
                else:
                    error_count += 1
            else:
                error_count += 1
            
            # Строгий тайминг: 250мкс пауза между пакетами
            pause_time = 0.00025  # 250мкс пауза
            start_time = time.perf_counter()
            
            # Ждем ровно 250мкс
            while time.perf_counter() - start_time < pause_time:
                pass
            
            # Получаем текущее время для логирования
            current_time = time.time()
            
            # Логирование статистики каждые 10 секунд
            if current_time - last_log_time >= 10:
                uart_waiting = rs485_transmitter.serial_port.out_waiting if rs485_transmitter.serial_port else 0
                # Вычисляем среднее время между пакетами
                current_packet_time = time.perf_counter()
                avg_interval = (current_packet_time - last_packet_time) / packet_count if packet_count > 0 else 0
                # Вычисляем реальную частоту
                real_frequency = packet_count / 10.0 if packet_count > 0 else 0
                # Вычисляем реальное время цикла
                cycle_time = (current_packet_time - last_packet_time) / packet_count if packet_count > 0 else 0
                logger.info(f"RS-485: {packet_count} пакетов, {error_count} ошибок, UART буфер: {uart_waiting} байт, цикл: {cycle_time*1000:.1f}мс, частота: {real_frequency:.1f} Гц")
                packet_count = 0
                error_count = 0
                last_log_time = current_time
                last_packet_time = current_packet_time
            
        except Exception as e:
            error_count += 1
            logger.error(f"Ошибка в задаче RS-485: {e}")
            time.sleep(0.001)  # Короткая пауза при ошибке

async def encoder_update_task(encoder: EncoderReader, data_store: ModbusDataStore):
    """Задача обновления данных энкодера в Modbus регистрах"""
    global counter
    
    while True:
        try:
            # Вычисляем углы
            ppr = data_store.get_pos_count_max()
            if ppr == 0:
                ppr = 360  # Значение по умолчанию
            
            # Безопасное чтение counter с блокировкой
            with counter_lock:
                current_counter = counter
            
            angle_rad = (current_counter % ppr) * (2 * math.pi / ppr)
            angle_deg = math.degrees(angle_rad)
            
            # Обновляем данные в Modbus регистрах
            data_store.update_encoder_data(current_counter, angle_rad, angle_deg)
            
            await asyncio.sleep(0.1)  # Обновляем каждые 100мс
            
        except Exception as e:
            logger.error(f"Ошибка в задаче обновления энкодера: {e}")
            await asyncio.sleep(0.1)

# ============================================================================
# ОСНОВНАЯ ФУНКЦИЯ
# ============================================================================

async def main():
    """Основная асинхронная функция"""
    global counter
    
    logger.info("=== Raspberry Pi 3 Encoder + Modbus Server + RS-485 ===")
    logger.info("Режим работы: симплексный (постоянная передача)")
    
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
        encoder = EncoderReader(pi_instance, GpioPins.ENCODER_A, GpioPins.ENCODER_B, GpioPins.ENCODER_Z)
        encoder.start()
        
        # Инициализация контроллера питания
        power_controller = PowerController(pi_instance, GpioPins.POWER_27V)
        power_controller.setup()
        
        # Инициализация RS-485 передатчика
        rs485_transmitter = RS485Transmitter(pi_instance, UartConfig.DEVICE, UartConfig.BAUDRATE, GpioPins.RS485_DE)
        rs485_transmitter.start()
        
        # Запуск Modbus сервера
        server_thread = run_modbus_server(data_store)
        
        logger.info("Все системы инициализированы. Начинаем работу...")
        logger.info("Modbus сервер запущен. Клиент может подключаться для настройки параметров.")
        
        # Запускаем RS-485 задачу в отдельном потоке
        rs485_thread = threading.Thread(target=rs485_transmission_task, args=(rs485_transmitter,), daemon=True)
        rs485_thread.start()
        
        # Запускаем остальные асинхронные задачи
        tasks = [
            asyncio.create_task(power_control_task(power_controller, data_store)),
            asyncio.create_task(parameter_update_task(data_store, rs485_transmitter)),
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

# ============================================================================
# ТОЧКА ВХОДА
# ============================================================================

if __name__ == "__main__":
    asyncio.run(main())