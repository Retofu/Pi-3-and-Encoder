import pigpio
import math
import time
import threading
import serial
import struct
import glob
import os

# Настройка пинов энкодера (из modbus.py)
A_PIN = 17  # Фаза A (GPIO17, pin 11)
B_PIN = 22  # Фаза B (GPIO22, pin 12) 
Z_PIN = 27  # Фаза Z (GPIO27, pin 13)

PPR = 1200  # Разрешение энкодера (импульсов на оборот)

# Настройки RS-485
RS485_BAUDRATE = 9600
RS485_TIMEOUT = 0.1

def find_serial_ports():
    """Поиск доступных последовательных портов"""
    ports = []
    
    # Поиск USB-адаптеров
    usb_ports = glob.glob('/dev/ttyUSB*')
    ports.extend(usb_ports)
    
    # Поиск ACM портов (USB CDC)
    acm_ports = glob.glob('/dev/ttyACM*')
    ports.extend(acm_ports)
    
    # Поиск других последовательных портов
    other_ports = glob.glob('/dev/ttyS*')
    ports.extend(other_ports)
    
    # Фильтруем только существующие порты
    existing_ports = [port for port in ports if os.path.exists(port)]
    
    return sorted(existing_ports)

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
        
        # Диагностика состояния пинов
        print("Диагностика энкодера:")
        print(f"  Пин A (GPIO{A_PIN}): {self.pi.read(A_PIN)}")
        print(f"  Пин B (GPIO{B_PIN}): {self.pi.read(B_PIN)}")
        print(f"  Пин Z (GPIO{Z_PIN}): {self.pi.read(Z_PIN)}")
        
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
            
            # Альтернативная логика энкодера (более надежная)
            # При переходе A с 0 на 1, если B=0, то +1, если B=1, то -1
            if level == 1:  # RISING edge на A
                if b == 0:
                    counter += 1
                else:
                    counter -= 1
            else:  # FALLING edge на A
                if b == 1:
                    counter += 1
                else:
                    counter -= 1
            
            # Выводим информацию о каждом изменении для диагностики (только первые 50 импульсов)
            if abs(counter) <= 50 and abs(counter) % 10 == 0 and counter != 0:
                print(f"Энкодер: A={a}, B={b}, Счетчик={counter}")
        except Exception as e:
            print(f"Ошибка в обработчике A: {e}")
            
    def _handle_Z(self, gpio, level, tick):
        """Обработчик индекса Z"""
        global counter
        if level == 1 and self.running:  # RISING
            counter = 0

class RS485Transmitter:
    """Класс для передачи данных через RS-485"""
    
    def __init__(self, device=None, baudrate=RS485_BAUDRATE):
        self.device = device
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        
    def start(self):
        """Инициализация RS-485 интерфейса"""
        # Если устройство не указано, ищем доступные порты
        if self.device is None:
            available_ports = find_serial_ports()
            if not available_ports:
                print("ВНИМАНИЕ: Не найдено доступных последовательных портов")
                print("Переходим в режим симуляции (данные только в консоль)")
                self.running = True
                return
            
            print("Доступные последовательные порты:")
            for i, port in enumerate(available_ports):
                print(f"  {i}: {port}")
            
            # Пробуем подключиться к первому найденному порту
            self.device = available_ports[0]
            print(f"Используем порт: {self.device}")
        
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
            print(f"Ошибка подключения к {self.device}: {e}")
            print("Переходим в режим симуляции (данные только в консоль)")
            self.running = True
    
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
        if not self.running:
            return False
            
        # Если нет подключения к порту, работаем в режиме симуляции
        if not self.serial_port or not self.serial_port.is_open:
            return True  # Возвращаем True для симуляции
        
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

def test_encoder(encoder, duration=10):
    """Тестирование энкодера в течение указанного времени"""
    global counter
    print(f"\n=== Тест энкодера на {duration} секунд ===")
    print("Крутите энкодер и наблюдайте за изменением счетчика...")
    
    start_time = time.time()
    last_counter = counter
    changes_count = 0
    max_counter = counter
    min_counter = counter
    
    while time.time() - start_time < duration:
        if counter != last_counter:
            print(f"Счетчик изменился: {last_counter} -> {counter}")
            changes_count += 1
            max_counter = max(max_counter, counter)
            min_counter = min(min_counter, counter)
            last_counter = counter
        time.sleep(0.1)
    
    print(f"Тест завершен. Финальный счетчик: {counter}")
    print(f"Количество изменений: {changes_count}")
    print(f"Максимальное значение: {max_counter}")
    print(f"Минимальное значение: {min_counter}")
    
    # Энкодер работает, если было хотя бы 5 изменений
    return changes_count >= 5

def main():
    """Основная функция"""
    global counter, angle_rad
    
    print("=== Raspberry Pi 3 Encoder + RS-485 Transmitter ===")
    
    # Инициализация энкодера
    encoder = EncoderReader()
    try:
        encoder.start()
        
        # Тест энкодера
        print("\nПроверка работы энкодера...")
        if not test_encoder(encoder, 5):
            print("ВНИМАНИЕ: Энкодер не реагирует на вращение!")
            print("Проверьте подключение пинов:")
            print(f"  A_PIN = GPIO{A_PIN} (физический pin 11)")
            print(f"  B_PIN = GPIO{B_PIN} (физический pin 15)")
            print(f"  Z_PIN = GPIO{Z_PIN} (физический pin 13)")
            print("Продолжаем работу в режиме симуляции...")
        else:
            print("✓ Энкодер работает корректно!")
        
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
                # Вывод информации о переданном пакете (каждые 100 пакетов для уменьшения спама)
                if counter % 100 == 0:
                    print(f"Пакет #{counter//100}: Угол={angle_rad:.3f} рад, Счетчик={counter}, "
                          f"Байты 56-59={packet[55:59].hex()}")
            else:
                print("Ошибка отправки пакета")
            
            # Дополнительная диагностика каждые 1000 пакетов
            if counter % 1000 == 0 and counter != 0:
                print(f"Диагностика: Счетчик={counter}, Угол={angle_rad:.3f} рад")
            
            # Интервал 3 мс
            time.sleep(0.003)
            
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        rs485.stop()
        encoder.stop()

if __name__ == "__main__":
    main()
