import pigpio
import math
import time
import threading
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock

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

# Настройка пинов энкодера
A_PIN = 17  # Фаза A (GPIO17, pin 11)
B_PIN = 22  # Фаза B (GPIO22, pin 12) 
Z_PIN = 27  # Фаза Z (GPIO27, pin 13)

PPR = 1200  # Разрешение энкодера (импульсов на оборот)

# Глобальные переменные для данных энкодера
counter = 0
angle_rad = 0.0
angle_deg = 0.0

# Настройка ModBus
MODBUS_PORT = 1502  # Изменили с 502 на 1502 (не требует root)
MODBUS_UNIT_ID = 1

# Регистры ModBus (Holding Registers, адреса 0-99)
REG_ANGLE_RAD = 0      # Угол в радианах (float32, 2 регистра)
REG_ANGLE_DEG = 2      # Угол в градусах (float32, 2 регистра) 
REG_COUNTER = 4        # Счетчик импульсов (int32, 2 регистра)
REG_PPR = 6           # PPR энкодера (uint16, 1 регистр)

class EncoderReader:
    def __init__(self):
        self.pi = None
        self.cb_a = None
        self.cb_z = None
        self.running = False
        
    def start(self):
        """Инициализация и запуск чтения энкодера"""
        global counter, status
        
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

class ModbusDataStore:
    """Класс для обновления данных в ModBus регистрах"""
    
    def __init__(self, store):
        self.store = store
        
    def update_registers(self):
        """Обновление регистров данными энкодера"""
        global counter, angle_rad, angle_deg, PPR
        
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
        
        # Обновление регистров (новая API pymodbus 3.x)
        self.store['hr'].setValues(REG_ANGLE_RAD, angle_rad_data)  # Holding registers
        self.store['hr'].setValues(REG_ANGLE_DEG, angle_deg_data)
        self.store['hr'].setValues(REG_COUNTER, counter_data)
        self.store['hr'].setValues(REG_PPR, [PPR])

def run_modbus_server():
    """Запуск ModBus TCP сервера"""
    # Создание хранилища данных (новая API pymodbus 3.x)
    store = {
        'di': ModbusSequentialDataBlock(0, [0]*100),  # Discrete Inputs
        'co': ModbusSequentialDataBlock(0, [0]*100),  # Coils
        'hr': ModbusSequentialDataBlock(0, [0]*100),  # Holding Registers
        'ir': ModbusSequentialDataBlock(0, [0]*100)   # Input Registers
    }
    
    data_store = ModbusDataStore(store)
    
    print(f"Запуск ModBus TCP сервера на порту {MODBUS_PORT}")
    print(f"Unit ID: {MODBUS_UNIT_ID}")
    print("Регистры:")
    print(f"  {REG_ANGLE_RAD}-{REG_ANGLE_RAD+1}: Угол в радианах (float32)")
    print(f"  {REG_ANGLE_DEG}-{REG_ANGLE_DEG+1}: Угол в градусах (float32)")
    print(f"  {REG_COUNTER}-{REG_COUNTER+1}: Счетчик импульсов (int32)")
    print(f"  {REG_PPR}: PPR энкодера (uint16)")
    
    # Запуск сервера в отдельном потоке
    def server_thread():
        StartTcpServer(store, address=("192.168.20.37", MODBUS_PORT))
    
    server_thread_obj = threading.Thread(target=server_thread, daemon=True)
    server_thread_obj.start()
    
    return data_store

def main():
    """Основная функция"""
    global counter, angle_rad, angle_deg
    
    print("=== Raspberry Pi 3 Encoder + ModBus TCP ===")
    
    # Инициализация энкодера
    encoder = EncoderReader()
    try:
        encoder.start()
    except Exception as e:
        print(f"Ошибка инициализации энкодера: {e}")
        return
    
    # Запуск ModBus сервера
    try:
        data_store = run_modbus_server()
    except Exception as e:
        print(f"Ошибка запуска ModBus сервера: {e}")
        encoder.stop()
        return
    
    print("Сервер запущен. Нажмите Ctrl+C для остановки")
    
    try:
        while True:
            # Обновление ModBus регистров
            data_store.update_registers()
            
            # Вывод в консоль
            print(f"Угол: {angle_rad:.3f} рад ({angle_deg:.1f}°), Счетчик: {counter}")
            
            time.sleep(0.1)  # Обновление каждые 100мс
            
    except KeyboardInterrupt:
        print("\nОстановка...")
    finally:
        encoder.stop()

if __name__ == "__main__":
    main()
