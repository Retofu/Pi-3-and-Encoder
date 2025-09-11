# Pi-3-and-Encoder-

## Описание
Скрипт читает квадратурный энкодер на Raspberry Pi 3 по линиям A/B и индексной линии Z, считает шаги и выводит угол в радианах. Для работы используется `pigpio` (демон + Python‑клиент). Пины по умолчанию:

- A: `GPIO17`
- B: `GPIO22` (перенесено с GPIO18, чтобы избежать конфликта с аудио ШИМ)
- Z: `GPIO27`

Разрешение энкодера задаётся константой `PPR` в `main.py`.

## Требования
- Raspberry Pi 3 с Raspberry Pi OS (Bullseye/Bookworm)
- Установленный системный демон `pigpiod`
- Python 3.9+ и `pip`

Примечание: На новых образах Raspberry Pi OS интерфейс GPIO через sysfs удалён, поэтому `RPi.GPIO` может выдавать ошибку «Failed to add edge detection». Этот проект использует `pigpio`, который работает корректно на современных ядрах.

## Быстрый старт (рекомендуется)
1. Установите и запустите демон pigpio:
   ```bash
   sudo apt update
   sudo apt install -y pigpio
   sudo systemctl enable --now pigpiod
   sudo systemctl status pigpiod | cat
   ```

2. (Опционально) Создайте виртуальное окружение и установите Python‑клиент `pigpio`:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade pip
   pip install pigpio
   ```

3. Подключите энкодер к `GPIO17` (A), `GPIO22` (B), `GPIO27` (Z). Линии подтянуты вверх в коде; для энкодеров с открытым коллектором желательно внешняя подтяжка 4.7–10 кОм к 3.3V.

4. Запустите скрипт:
   ```bash
   python3 main.py
   ```
   sudo не требуется — `pigpiod` уже работает с правами root.

## Настройка
- Измените `PPR` в `main.py` под ваш энкодер.
- При необходимости переназначьте пины `A_PIN`, `B_PIN`, `Z_PIN` в начале файла.

## Проверка работы
- Команда должна выводить текущий угол и счётчик:
  ```
  Угол: 1.571 рад, Счетчик: 300
  ```
- Вращение по/против часовой стрелки изменяет знак приращения счётчика.

## Частые проблемы
- «ModuleNotFoundError: No module named 'pigpio'»:
  - Активируйте venv и выполните `pip install pigpio`.
  - Либо установите системный пакет: `sudo apt install -y python3-pigpio`.

- «Can’t connect to pigpio daemon»:
  - Запустите демон: `sudo systemctl start pigpiod`.
  - Проверьте статус: `sudo systemctl status pigpiod | cat`.

- Конфликт с аудио ШИМ на `GPIO18`:
  - В этом проекте B‑линия использует `GPIO22`. Если всё же нужно `GPIO18`, отключите аудио: в `/boot/config.txt` установите `dtparam=audio=off` и перезагрузите.

- Диагностика уровней пинов:
  ```bash
  raspi-gpio get 17 22 27
  ```
  Ожидается `func=INPUT` и `pull=UP`.

## Лицензия
MIT (если не указано иное).