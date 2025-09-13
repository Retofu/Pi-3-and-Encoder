# Pi-3-and-Encoder-

## Описание
Проект содержит два скрипта для работы с квадратурным энкодером на Raspberry Pi 3:

- **`main.py`** — базовое чтение энкодера с выводом в консоль
- **`modbus.py`** — чтение энкодера + ModBus TCP/IP сервер для удалённого доступа к данным

Пины по умолчанию:
- A: `GPIO17`
- B: `GPIO22` (перенесено с GPIO18, чтобы избежать конфликта с аудио ШИМ)
- Z: `GPIO27`

Разрешение энкодера задаётся константой `PPR` в скриптах.

## Требования
- Raspberry Pi 3 с Raspberry Pi OS (Bullseye/Bookworm)
- Установленный системный демон `pigpiod`
- Python 3.9+ и `pip`

### Зависимости
- **`main.py`**: `pigpio` (Python-клиент)
- **`modbus.py`**: `pigpio` + `pymodbus` (версия 3.11.2)

Примечание: На новых образах Raspberry Pi OS интерфейс GPIO через sysfs удалён, поэтому `RPi.GPIO` может выдавать ошибку «Failed to add edge detection». Этот проект использует `pigpio`, который работает корректно на современных ядрах.

## Быстрый старт (рекомендуется)
1. Установите и запустите демон pigpio:
   ```bash
   sudo apt update
   sudo apt install -y pigpio
   sudo systemctl enable --now pigpiod
   sudo systemctl status pigpiod | cat
   ```

2. (Опционально) Создайте виртуальное окружение и установите зависимости:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade pip
   
   # Для main.py:
   pip install pigpio
   
   # Для modbus.py:
   pip install pigpio pymodbus==3.11.2
   ```

3. Подключите энкодер к `GPIO17` (A), `GPIO22` (B), `GPIO27` (Z). Линии подтянуты вверх в коде; для энкодеров с открытым коллектором желательно внешняя подтяжка 4.7–10 кОм к 3.3V.

4. Запустите скрипт:
   ```bash
   # Базовое чтение энкодера:
   python3 main.py
   
   # Чтение энкодера + ModBus TCP сервер:
   python3 modbus.py
   ```
   sudo не требуется — `pigpiod` уже работает с правами root.

## Настройка
- Измените `PPR` в скриптах под ваш энкодер.
- При необходимости переназначьте пины `A_PIN`, `B_PIN`, `Z_PIN` в начале файлов.
- Для `modbus.py`: измените `MODBUS_PORT` (по умолчанию 2502) и `MODBUS_UNIT_ID` (по умолчанию 1).

## ModBus TCP регистры (modbus.py)
Сервер предоставляет следующие регистры:

### Holding Registers (команды 03, 06, 16)
| Адрес | Тип | Описание |
|-------|-----|----------|
| 0-1 | float32 | Угол в радианах |
| 2-3 | float32 | Угол в градусах |
| 4-5 | int32 | Счетчик импульсов |
| 6 | uint16 | PPR энкодера |

### Input Registers (команда 04)
| Адрес | Тип | Описание |
|-------|-----|----------|
| 0-1 | float32 | Угол в радианах |
| 2-3 | float32 | Угол в градусах |
| 4-5 | int32 | Счетчик импульсов |
| 6 | uint16 | PPR энкодера |

**Подключение к серверу**: `IP_адрес_Pi:2502`, Unit ID: 1

**Примечание**: Данные дублируются в Holding и Input регистрах для совместимости с различными ModBus клиентами.

## Проверка работы
- **main.py** должен выводить:
  ```
  Угол: 1.571 рад, Счетчик: 300
  ```
- **modbus.py** выводит данные энкодера + запускает ModBus сервер на порту 2502
- Вращение по/против часовой стрелки изменяет знак приращения счётчика

## Частые проблемы

### Ошибки импорта
- **«ModuleNotFoundError: No module named 'pigpio'»**:
  - Активируйте venv и выполните `pip install pigpio`.
  - Либо установите системный пакет: `sudo apt install -y python3-pigpio`.

- **«ModuleNotFoundError: No module named 'pymodbus'»**:
  - Активируйте venv и выполните `pip install pymodbus==3.11.2`.

### Ошибки ModBus сервера
- **«KeyError: 1» или «TypeError: 'ModbusSequentialDataBlock' object is not subscriptable»**:
  - Убедитесь, что используется pymodbus версии 3.11.2: `pip install pymodbus==3.11.2`.
  - Код обновлен для совместимости с новой версией API.

- **«Address already in use» (порт 2502)**:
  - Измените `MODBUS_PORT` в `modbus.py` на свободный порт (например, 3502).
  - Или остановите другие ModBus серверы: `sudo netstat -tlnp | grep :2502`.

### Ошибки pigpio
- **«Can't connect to pigpio daemon»**:
  - Запустите демон: `sudo systemctl start pigpiod`.
  - Проверьте статус: `sudo systemctl status pigpiod | cat`.

### Проблемы с GPIO
- **Конфликт с аудио ШИМ на `GPIO18`**:
  - В этом проекте B‑линия использует `GPIO22`. Если всё же нужно `GPIO18`, отключите аудио: в `/boot/config.txt` установите `dtparam=audio=off` и перезагрузите.

- **Диагностика уровней пинов**:
  ```bash
  raspi-gpio get 17 22 27
  ```
  Ожидается `func=INPUT` и `pull=UP`.

### Совместимость с pymodbus 3.11.2
Код обновлен для работы с pymodbus 3.11.2, который имеет кардинально измененный API:
- Используется `ModbusDeviceContext` вместо `ModbusSlaveContext`
- Параметр `devices={}` вместо `slaves={}`
- Поддержка как Holding, так и Input регистров для максимальной совместимости

## Лицензия
MIT (если не указано иное).