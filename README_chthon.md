# Raspberry Pi 3 Encoder + Modbus Server + RS-485 Transmitter

Асинхронный скрипт для Raspberry Pi 3, который работает как Modbus сервер, читает данные с энкодера и передает их по RS-485.

## Функциональность

- **Чтение энкодера**: Обработка сигналов с инкрементального энкодера (пины GPIO17, GPIO22, GPIO27)
- **Modbus сервер**: Предоставление данных энкодера и получение команд от клиента (Unit ID 255)
- **RS-485 передача**: Отправка пакетов 120 байт с углом крена и статусным словом
- **Управление питанием**: Контроль питания 27В через GPIO25 по команде от Modbus клиента
- **Асинхронная работа**: Параллельная работа Modbus сервера и RS-485 с использованием asyncio

## Установка зависимостей

```bash
# Установка Python пакетов
pip install -r requirements.txt

# Установка pigpio daemon (если не установлен)
sudo apt-get update
sudo apt-get install pigpio

# Запуск pigpio daemon
sudo systemctl enable pigpio
sudo systemctl start pigpio
```

## Структура проекта

- `chthon.py` - Основной скрипт сервера (включает все необходимые классы)
- `modbus_client_example.py` - Пример клиента для тестирования
- `requirements.txt` - Зависимости Python

## Встроенные классы

Скрипт `chthon.py` включает следующие встроенные классы для работы с Modbus:

### Endian
```python
class Endian:
    Big = 0      # Big-endian (стандарт Modbus)
    Little = 1   # Little-endian
```

### BinaryPayloadBuilder
Класс для упаковки данных в Modbus регистры:
```python
builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
builder.add_32bit_float(3.14159)  # Добавить float32
builder.add_32bit_int(-12345)     # Добавить int32
registers = builder.to_registers() # Получить список регистров
```

### BinaryPayloadDecoder
Класс для распаковки данных из Modbus регистров:
```python
decoder = BinaryPayloadDecoder(registers, byteorder=Endian.Big, wordorder=Endian.Big)
float_value = decoder.decode_32bit_float()  # Декодировать float32
int_value = decoder.decode_32bit_int()      # Декодировать int32
uint_value = decoder.decode_16bit_uint()    # Декодировать uint16
```

**Примечание**: Все классы встроены в `chthon.py` и не требуют дополнительных файлов.

## Настройка UART

1. Включите UART в `/boot/config.txt`:
```
enable_uart=1
dtoverlay=pi3-miniuart-bt
```

2. Отключите Bluetooth (опционально):
```
dtoverlay=pi3-disable-bt
```

3. Перезагрузите Raspberry Pi:
```bash
sudo reboot
```

## Настройка пинов

Скрипт использует следующие пины:
- **GPIO17** (Pin 11): Фаза A энкодера
- **GPIO22** (Pin 15): Фаза B энкодера  
- **GPIO27** (Pin 13): Фаза Z энкодера (индекс)
- **GPIO24** (Pin 18): DE пин для RS-485
- **GPIO25** (Pin 22): Управление питанием 27В

## Конфигурация

Отредактируйте настройки в начале файла `chthon.py`:

```python
# Настройки Modbus сервера
MODBUS_SERVER_PORT = 502
MODBUS_UNIT_ID = 255

# Настройки UART
UART_DEVICE = '/dev/serial0'
UART_BAUDRATE = 507000
```

## Подключение Modbus клиента

Плата работает как Modbus сервер на порту 502. Для подключения с компьютера используйте любой Modbus клиент:

- **IP адрес**: IP адрес Raspberry Pi
- **Порт**: 502
- **Unit ID**: 255

## Структура RS-485 пакета

Пакет состоит из 120 байт:
- **Байт 0**: Постоянный заголовок (0x65)
- **Байты 1-55**: Нули
- **Байты 56-59**: Угол крена в радианах (float, little-endian)
- **Байты 60-80**: Нули
- **Байты 81-82**: Статусное слово (uint16, little-endian)
- **Байты 83-116**: Нули
- **Байт 117**: Контрольная сумма (CS = 0xFF - (0xFF & Σᵢ СДᵢ))
- **Байт 118**: Постоянный заголовок (0x45)
- **Байт 119**: Постоянный заголовок (0xCF)

## Формула расчета угла

```
Angle_roll = (Mod360(Offset_angle_roll + angle_encoder)) * π / 180
```

Где:
- `Offset_angle_roll` - смещение угла из Modbus (SP_Angle_Offset)
- `angle_encoder` - угол энкодера в градусах
- `Mod360` - приведение к диапазону 0-360 градусов

## Формула контрольной суммы

Контрольная сумма вычисляется по формуле:
```
CS = 0xFF - (0xFF & Σᵢ СДᵢ)
```

Где:
- `СДᵢ` - слова сообщения (байты 0-116)
- `Σᵢ СДᵢ` - сумма всех байтов от 0 до 116 включительно
- `&` - побитовая операция AND
- `0xFF` - константа 255 (0xFF в шестнадцатеричном)

## Modbus регистры

### Регистры обратной связи (1000-1019) - только чтение
- **1000**: FBK_Delay_Before - пауза до формирования импульса (UINT)
- **1001**: FBK_Delay_Count - текущее время паузы (UINT)
- **1002**: FBK_Pos_Count - количество импульсов от энкодера (UINT)
- **1003**: FBK_Pos_Count_Max - максимальное количество имп/об (UINT)
- **1004**: FBK_Pos - угол поворота в градусах (UINT)
- **1005**: FBK_Pulse_Length - длительность сработки выхода (UINT)
- **1006**: FBK_Pulse_Count - текущее время активности выхода (UINT)
- **1007**: FBK_Front_Type - тип детектирования нуль-точки (UINT)
- **1008**: FBK_Pulse_On - состояние выхода нуль-точки (UINT)
- **1009**: FBK_Power_27_V - состояние питания 27В (UINT)
- **1010**: FBK_POS_Set - команда записи позиции (UINT)
- **1011**: FBK_Reserve_11 - резерв (UINT)
- **1012-1013**: FBK_Angle_Roll - угол крена в радианах (FLOAT)
- **1014-1015**: FBK_Angle_Adj - угол юстировки в радианах (FLOAT)
- **1016-1019**: Резервные регистры (UINT)

### Регистры задания (2000-2013) - чтение/запись
- **2000**: SP_Delay_Before - задание паузы до импульса (UINT)
- **2001**: SP_reserve_1 - резерв (UINT)
- **2002**: SP_Pos_Count - текущее количество импульсов (UINT)
- **2003**: SP_Pos_Count_Max - максимальное количество имп/об (UINT)
- **2004**: SP_reserve_4 - резерв (UINT)
- **2005**: SP_Pulse_Length - задание длительности выхода (UINT)
- **2006**: SP_reserve_6 - резерв (UINT)
- **2007**: SP_Front_Type - задание типа детектирования (UINT)
- **2008**: SP_Pulse_On - команда выхода нуль-точки (UINT)
- **2009**: SP_Power_27_V - команда питания 27В (UINT) ⚡
- **2010**: SP_POS_Set - команда записи позиции (UINT)
- **2011**: SP_Status - статусное слово (UINT)
- **2012-2013**: SP_Angle_Offset - смещение угла в градусах (FLOAT) ⚡

## Запуск

```bash
# Запуск с правами root (необходимо для GPIO)
sudo python3 chthon.py
```

## Логирование

Скрипт выводит подробную информацию о работе:
- Подключение к Modbus устройству
- Чтение переменных
- Состояние питания 27В
- Ошибки передачи RS-485

## Остановка

Нажмите `Ctrl+C` для корректной остановки скрипта.

## Устранение неполадок

1. **Ошибка "pigpio daemon is not running"**:
   ```bash
   sudo systemctl start pigpio
   ```

2. **Ошибка подключения к Modbus серверу**:
   - Проверьте, что сервер запущен на порту 502
   - Убедитесь, что порт не заблокирован файрволом
   - Проверьте IP адрес Raspberry Pi

3. **Ошибка UART**:
   - Проверьте настройки в `/boot/config.txt`
   - Убедитесь, что UART не используется другими процессами

4. **Ошибки GPIO**:
   - Запустите скрипт с правами root
   - Проверьте, что пины не используются другими процессами
