import pigpio
import math
import time
import threading
import serial
import struct

# Настройка пинов энкодера (из modbus.py)
A_PIN = 17  # Фаза A (GPIO17, pin 11)
B_PIN = 22  # Фаза B (GPIO22, pin 12) 
Z_PIN = 27  # Фаза Z (GPIO27, pin 13)

PPR = 1200  # Разрешение энкодера (импульсов на оборот)

# Настройки RS-485
RS485_DEVICE = '/dev/ttyUSB0'  # Путь к устройству RS-485
RS485_BAUDRATE = 9600
RS485_TIMEOUT = 0.1

# Глобальные переменные для данных энкодера
counter = 0
angle_rad = 0.0

class EncoderReader:
    """Класс для чтения данных с энкодера (адаптирован из modbus.py)"""
    
    def __init__(self):
        self.pi = None
        self.cb_a = None
        self.cb_z = None
        self.running = False
        
    def start(self):
        """Инициализация и запуск чтения энкодера"""
        global counter
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running")
            
        # Настройка пинов
        self.pi.set_mode(A_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(A_PIN, pigpio.PUD_UP)
        self.pi.set_mode(B_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(B_PIN, pigpio.PUD_UP)
        self.pi.set_mode(Z_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(Z_PIN, pigpio.PUD_UP)
        
        # Фильтр дребезга
        self.pi.set_glitch_filter(A_PIN, 200)
        self.pi.set_glitch_filter(B_PIN, 200)
        self.pi.set_glitch_filter(Z_PIN, 200)
        
        # Обработчики прерываний
        self.cb_a = self.pi.callback(A_PIN, pigpio.EITHER_EDGE, self._handle_A)
        self.cb_z = self.pi.callback(Z_PIN, pigpio.RISING_EDGE, self._handle_Z)
        
        self.running = True
        print("Энкодер инициализирован")
        
    def stop(self):
        """Остановка чтения энкодера"""
        self.running = False
        if self.cb_a:
            self.cb_a.cancel()
        if self.cb_z:
            self.cb_z.cancel()
        if self.pi:
            self.pi.stop()
        print("Энкодер остановлен")
        
    def _handle_A(self, gpio, level, tick):
        """Обработчик фазы A"""
        global counter
        if level == pigpio.TIMEOUT or not self.running:
            return
        try:
            a = self.pi.read(A_PIN)
            b = self.pi.read(B_PIN)
            counter += 1 if a == b else -1
        except:
            pass
            
    def _handle_Z(self, gpio, level, tick):
        """Обработчик индекса Z"""
        global counter
        if level == 1 and self.running:  # RISING
            counter = 0

class RS485Transmitter:
    """Класс для передачи данных через RS-485"""
    
    def __init__(self, device=RS485_DEVICE, baudrate=RS485_BAUDRATE):
        self.device = device
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        
    def start(self):
        """Инициализация RS-485 интерфейса"""
        try:
            self.serial_port = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=RS485_TIMEOUT
            )
            self.running = True
            print(f"RS-485 инициализирован: {self.device}, {self.baudrate} bps")
        except Exception as e:
            raise RuntimeError(f"Ошибка инициализации RS-485: {e}")
    
    def stop(self):
        """Остановка RS-485 интерфейса"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        print("RS-485 остановлен")
    
    def create_data_packet(self, angle_rad):
        """
        Создание пакета данных размером 116 байт
        Байты 1-55 и 60-116: 0
        Байты 56-59: угол в радианах (float32, little-endian)
        """
        packet = bytearray(116)
        
        # Байты 1-55: заполняем нулями (индексы 0-54)
        for i in range(55):
            packet[i] = 0
        
        # Байты 56-59: угол в радианах как float32 (little-endian)
        # Используем struct.pack для упаковки float32 в little-endian
        angle_bytes = struct.pack('<f', angle_rad)
        packet[55:59] = angle_bytes  # Байты 56-59 (индексы 55-58)
        
        # Байты 60-116: заполняем нулями (индексы 59-115)
        for i in range(59, 116):
            packet[i] = 0
        
        return packet
    
    def send_packet(self, packet):
        """Отправка пакета данных через RS-485"""
        if not self.running or not self.serial_port or not self.serial_port.is_open:
            return False
        
        try:
            self.serial_port.write(packet)
            self.serial_port.flush()  # Обеспечиваем немедленную отправку
            return True
        except Exception as e:
            print(f"Ошибка отправки пакета: {e}")
            return False

def update_angle():
    """Обновление угла в радианах (адаптировано из modbus.py)"""
    global counter, angle_rad
    
    # Расчет угла в радианах (точно так же, как в modbus.py)
    angle_rad = (counter % PPR) * (2 * math.pi / PPR)

def main():
    """Основная функция"""
    global counter, angle_rad
    
    print("=== Raspberry Pi 3 Encoder + RS-485 Transmitter ===")
    
    # Инициализация энкодера
    encoder = EncoderReader()
    try:
        encoder.start()
    except Exception as e:
        print(f"Ошибка инициализации энкодера: {e}")
        return
    
    # Инициализация RS-485
    rs485 = RS485Transmitter()
    try:
        rs485.start()
    except Exception as e:
        print(f"Ошибка инициализации RS-485: {e}")
        encoder.stop()
        return
    
    print("Система запущена. Передача данных каждые 3 мс. Нажмите Ctrl+C для остановки")
    
    try:
        while True:
            # Обновление угла
            update_angle()
            
            # Создание пакета данных
            packet = rs485.create_data_packet(angle_rad)
            
            # Отправка пакета
            if rs485.send_packet(packet):
                # Вывод информации о переданном пакете
                print(f"Отправлен пакет: Угол={angle_rad:.3f} рад, Счетчик={counter}, "
                      f"Байты 56-59={packet[55:59].hex()}")
            else:
                print("Ошибка отправки пакета")
            
            # Интервал 3 мс
            time.sleep(0.003)
            
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        rs485.stop()
        encoder.stop()

if __name__ == "__main__":
    main()
