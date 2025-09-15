import pigpio
import logging
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.transaction import ModbusSocketFramer

# Настройка логирования
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Настройка GPIO
GPIO_PIN = 25  # GPIO 25 для дискретного выхода

# Настройка ModBus
MODBUS_PORT = 2502  # Стандартный порт Modbus TCP
MODBUS_UNIT_ID = 1

# Адрес регистра для команды 06
REGISTER_ADDRESS = 2009

class GPIOController:
    """Класс для управления GPIO пином"""
    
    def __init__(self, pin):
        self.pin = pin
        self.pi = None
        
    def initialize(self):
        """Инициализация GPIO"""
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running")
            
        # Настройка пина как выход
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.write(self.pin, 0)  # Устанавливаем начальное значение 0
        logger.info(f"GPIO {self.pin} инициализирован как выход")
        
    def set_output(self, value):
        """Установка значения на выходе"""
        if self.pi:
            self.pi.write(self.pin, value)
            logger.info(f"GPIO {self.pin} установлен в значение: {value}")
        else:
            logger.error("GPIO не инициализирован")
            
    def cleanup(self):
        """Очистка ресурсов"""
        if self.pi:
            self.pi.write(self.pin, 0)  # Устанавливаем 0 перед выходом
            self.pi.stop()
            logger.info("GPIO очищен")

class ModbusDataStore:
    """Класс для обработки данных Modbus"""
    
    def __init__(self, gpio_controller):
        self.gpio_controller = gpio_controller
        
    def handle_write_single_register(self, address, value):
        """Обработка команды 06 (Write Single Register)"""
        logger.info(f"Получена команда 06: адрес={address}, значение={value}")
        
        if address == REGISTER_ADDRESS:
            # Обрабатываем только адрес 2009
            if value == 0:
                self.gpio_controller.set_output(0)
                logger.info("GPIO 25 установлен в 0")
            elif value == 1:
                self.gpio_controller.set_output(1)
                logger.info("GPIO 25 установлен в 1")
            else:
                logger.warning(f"Недопустимое значение {value} для адреса {address}")
        else:
            logger.warning(f"Неизвестный адрес регистра: {address}")

class ModbusServer:
    """Класс Modbus сервера с кастомной обработкой команд"""
    
    def __init__(self, gpio_controller):
        self.gpio_controller = gpio_controller
        self.data_store = ModbusDataStore(gpio_controller)
        
        # Создаем хранилище данных
        self.hr_block = ModbusSequentialDataBlock(0, [0] * 10000)  # Holding Registers
        self.ir_block = ModbusSequentialDataBlock(0, [0] * 10000)  # Input Registers
        self.di_block = ModbusSequentialDataBlock(0, [0] * 10000)  # Discrete Inputs
        self.co_block = ModbusSequentialDataBlock(0, [0] * 10000)  # Coils
        
        # Создаем контекст устройства
        self.device_context = ModbusSlaveContext(
            di=self.di_block,
            co=self.co_block,
            hr=self.hr_block,
            ir=self.ir_block
        )
        
        # Создаем контекст сервера
        self.server_context = ModbusServerContext(
            devices={MODBUS_UNIT_ID: self.device_context}, 
            single=False
        )
        
    def start_server(self):
        """Запуск Modbus сервера"""
        # Настройка идентификации устройства
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'Raspberry Pi'
        identity.ProductCode = 'GPIO_Controller'
        identity.VendorUrl = 'https://www.raspberrypi.org/'
        identity.ProductName = 'GPIO Modbus Server'
        identity.ModelName = 'Pi3 GPIO Controller'
        identity.MajorMinorRevision = '1.0'
        
        logger.info(f"Запуск Modbus TCP сервера на порту {MODBUS_PORT}")
        logger.info(f"Unit ID: {MODBUS_UNIT_ID}")
        logger.info(f"Ожидание команды 06 по адресу регистра {REGISTER_ADDRESS}")
        
        # Запуск сервера
        StartTcpServer(
            self.server_context,
            identity=identity,
            address=("0.0.0.0", MODBUS_PORT),
            framer=ModbusSocketFramer
        )

def main():
    """Основная функция"""
    logger.info("=== Raspberry Pi 3 Modbus GPIO Server ===")
    
    # Инициализация GPIO контроллера
    gpio_controller = GPIOController(GPIO_PIN)
    
    try:
        # Инициализация GPIO
        gpio_controller.initialize()
        
        # Создание и запуск Modbus сервера
        modbus_server = ModbusServer(gpio_controller)
        
        # Переопределяем метод обработки записи в регистры
        original_setValues = modbus_server.hr_block.setValues
        
        def custom_setValues(address, values):
            """Кастомная обработка записи в регистры"""
            logger.info(f"Запись в регистр {address}: {values}")
            
            # Обрабатываем команду 06 (запись одного регистра)
            if len(values) == 1:
                modbus_server.data_store.handle_write_single_register(address, values[0])
            
            # Вызываем оригинальный метод
            return original_setValues(address, values)
        
        # Заменяем метод
        modbus_server.hr_block.setValues = custom_setValues
        
        logger.info("Сервер готов к работе. Нажмите Ctrl+C для остановки")
        
        # Запуск сервера (блокирующий вызов)
        modbus_server.start_server()
        
    except KeyboardInterrupt:
        logger.info("Получен сигнал остановки...")
    except Exception as e:
        logger.error(f"Ошибка: {e}")
    finally:
        # Очистка ресурсов
        gpio_controller.cleanup()
        logger.info("Сервер остановлен")

if __name__ == "__main__":
    main()