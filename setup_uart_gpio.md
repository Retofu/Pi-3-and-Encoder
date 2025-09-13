# Настройка UART на GPIO14/15 для RS-485

## Шаг 1: Настройка /boot/config.txt

Отредактируйте файл конфигурации:
```bash
sudo nano /boot/config.txt
```

Добавьте или раскомментируйте следующие строки:
```
# Включить UART
enable_uart=1

# Переназначить UART на GPIO14/15 (вместо GPIO32/33)
dtoverlay=uart0,txd0_pin=14,rxd0_pin=15

# Отключить Bluetooth (освобождает GPIO14/15)
dtoverlay=disable-bt
```

## Шаг 2: Отключение Bluetooth

Если вы используете `dtoverlay=disable-bt`, то Bluetooth будет отключен. Если нужно сохранить Bluetooth, используйте альтернативные пины.

## Шаг 3: Перезагрузка

```bash
sudo reboot
```

## Шаг 4: Проверка настройки

После перезагрузки проверьте:
```bash
# Проверить, что UART доступен
ls -l /dev/serial0

# Проверить статус Bluetooth (должен быть отключен)
systemctl status bluetooth
```

## Шаг 5: Запуск скрипта

```bash
python3 rs_uart_gpio.py
```

## Подключение к логическому анализатору

```
Raspberry Pi    Логический анализатор
GPIO14 (pin 8)  → Канал 1 (TX)
GPIO15 (pin 10) → Канал 2 (RX)
GPIO23 (pin 16) → Канал 3 (DE/RE)
GND            → GND
```

## Ожидаемый результат

При скорости 436,364 bps:
- **Время передачи 120 байт**: ~2.75 мс
- **Пауза**: 0.25 мс
- **Общий цикл**: 3 мс

## Альтернативная настройка (если нужен Bluetooth)

Если нужно сохранить Bluetooth, используйте mini UART:
```
# В /boot/config.txt
enable_uart=1
dtoverlay=miniuart-bt
dtoverlay=uart0,txd0_pin=14,rxd0_pin=15
```

## Устранение проблем

### Ошибка "Permission denied"
```bash
sudo usermod -a -G dialout $USER
# Перелогиньтесь или перезагрузитесь
```

### UART не работает
```bash
# Проверить статус UART
sudo systemctl status serial-getty@ttyS0
sudo systemctl disable serial-getty@ttyS0
```

### Проверить доступные устройства
```bash
ls -l /dev/tty*
```
