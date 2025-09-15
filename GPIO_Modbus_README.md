# Raspberry Pi 3 Modbus GPIO Server

Этот скрипт превращает Raspberry Pi 3 в Modbus TCP сервер, который управляет GPIO 25 в зависимости от команд, получаемых от компьютера.

## Функциональность

- Raspberry Pi 3 работает как Modbus TCP сервер
- Принимает команду 06 (Write Single Register) по адресу регистра 2009
- Управляет GPIO 25:
  - Значение 0 → GPIO 25 = 0 (низкий уровень)
  - Значение 1 → GPIO 25 = 1 (высокий уровень)
- Логирование всех операций для отладки

## Установка зависимостей

```bash
# Установка Python пакетов (pymodbus 3.11.2)
pip install -r requirements.txt

# Установка pigpio daemon (если не установлен)
sudo apt-get update
sudo apt-get install pigpio

# Запуск pigpio daemon
sudo systemctl enable pigpio
sudo systemctl start pigpio
```

## Запуск

```bash
python modbus.py
```

## Использование

1. Запустите скрипт на Raspberry Pi
2. С компьютера отправляйте Modbus команды:
   - Команда 06 (Write Single Register)
   - Адрес регистра: 2009
   - Значение: 0 или 1

## Примеры команд

### Включить GPIO 25 (установить в 1)
```
Function Code: 06
Register Address: 2009
Value: 1
```

### Выключить GPIO 25 (установить в 0)
```
Function Code: 06
Register Address: 2009
Value: 0
```

## Логирование

Скрипт выводит подробную информацию о:
- Полученных Modbus командах
- Изменениях состояния GPIO
- Ошибках и предупреждениях

## Технические детали

- **Порт Modbus TCP**: 502 (стандартный)
- **Unit ID**: 1
- **GPIO пин**: 25
- **Адрес регистра**: 2009
- **Поддерживаемые команды**: 06 (Write Single Register)

## Остановка

Нажмите `Ctrl+C` для корректной остановки сервера.
