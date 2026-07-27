#!/usr/bin/env python3
"""
Чтение данных с датчика Waveshare TOF Laser Range Sensor Mini (SKU 28761)
через UART. Датчик отправляет бинарные пакеты по 16 байт
в формате протокола NLink_TOFSense_Frame0 (не текст!).

Формат пакета (little-endian):
  [0]     Frame Header      = 0x57
  [1]     Function Mark     = 0x00
  [2]     reserved          = 0xff
  [3]     id
  [4:8]   System_time (мс), uint32
  [8:11]  dis*1000 (мм), uint24 (3 байта)
  [11]    dis_status  (0 = невалидно, 1 = валидно)
  [12:14] signal_strength, uint16
  [14]    range_precision (см)
  [15]    Sum Check (контрольная сумма)

РЕЖИМЫ РАБОТЫ (задаются переменной MODE ниже):
  1 - Подсчёт мин / макс / среднего за последние 100 измерений
  2 - Определение «объект ближе 200 мм»
  3 - Определение «объект ближе 1.5 м»
  4 - Фильтр выбросов + скользящее среднее
"""

import serial
import time
from collections import deque

# ==================== НАСТРОЙКИ ====================
UART_PORT = "/dev/serial0"
BAUDRATE = 921600
TIMEOUT = 1
FRAME_LEN = 16
FRAME_HEADER = 0x57
FUNCTION_MARK = 0x00

WINDOW_SIZE = 100          # для режима 1 и 4 (скользящее окно)
THRESHOLD_MM_200 = 200     # для режима 2
THRESHOLD_M_1_5 = 1.5      # для режима 3
OUTLIER_JUMP_M = 0.5       # для режима 4: макс. допустимый скачок между соседними измерениями (м)
# =====================================================


def open_uart(port: str, baudrate: int, timeout: float = 1) -> serial.Serial:
    uart = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    print(f"UART порт {port} открыт на скорости {baudrate} бод")
    return uart


def checksum_ok(frame: bytes) -> bool:
    """Контрольная сумма = младший байт суммы первых 15 байт."""
    return (sum(frame[:-1]) & 0xFF) == frame[-1]


def parse_frame(frame: bytes):
    """Разбирает 16-байтный пакет NLink_TOFSense_Frame0."""
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != FRAME_HEADER or frame[1] != FUNCTION_MARK:
        return None
    if not checksum_ok(frame):
        return None

    module_id = frame[3]
    system_time = int.from_bytes(frame[4:8], byteorder="little")
    dis_raw = int.from_bytes(frame[8:11], byteorder="little")  # uint24
    distance_m = dis_raw / 1000.0
    dis_status = frame[11]
    signal_strength = int.from_bytes(frame[12:14], byteorder="little")
    range_precision = frame[14]

    return {
        "id": module_id,
        "time_ms": system_time,
        "distance_m": distance_m,
        "valid": dis_status == 1,
        "signal_strength": signal_strength,
        "precision_cm": range_precision,
    }


# ==================== ЛОГИКА РЕЖИМОВ ====================

class ModeStats:
    """Режим 1: мин / макс / среднее за последние WINDOW_SIZE измерений."""

    def __init__(self, window_size: int):
        self.window = deque(maxlen=window_size)

    def process(self, result: dict):
        if not result["valid"]:
            return
        self.window.append(result["distance_m"])

        d_min = min(self.window)
        d_max = max(self.window)
        d_avg = sum(self.window) / len(self.window)

        print(
            f"Дистанция: {result['distance_m']:.3f} м | "
            f"Окно: {len(self.window)}/{self.window.maxlen} | "
            f"Мин: {d_min:.3f} м | Макс: {d_max:.3f} м | Среднее: {d_avg:.3f} м"
        )


class ModeThreshold:
    """Режимы 2 и 3: обнаружение объекта ближе порога."""

    def __init__(self, threshold_m: float, label: str):
        self.threshold_m = threshold_m
        self.label = label
        self.was_close = False  # чтобы не спамить одинаковыми сообщениями

    def process(self, result: dict):
        if not result["valid"]:
            return

        is_close = result["distance_m"] < self.threshold_m

        if is_close and not self.was_close:
            print(f"⚠ ОБЪЕКТ ОБНАРУЖЕН ({self.label})! Дистанция: {result['distance_m']:.3f} м")
        elif not is_close and self.was_close:
            print(f"Объект вышел за пределы {self.label}. Дистанция: {result['distance_m']:.3f} м")
        else:
            print(f"Дистанция: {result['distance_m']:.3f} м | Близко ({self.label}): {is_close}")

        self.was_close = is_close


class ModeFilteredAverage:
    """Режим 4: отбрасывает резкие выбросы и считает скользящее среднее."""

    def __init__(self, window_size: int, max_jump_m: float):
        self.window = deque(maxlen=window_size)
        self.max_jump_m = max_jump_m
        self.last_accepted = None

    def process(self, result: dict):
        if not result["valid"]:
            return

        distance = result["distance_m"]

        # Проверка на выброс: слишком большой скачок относительно последнего принятого значения
        if self.last_accepted is not None and abs(distance - self.last_accepted) > self.max_jump_m:
            print(f"Выброс отфильтрован: {distance:.3f} м (пред. значение: {self.last_accepted:.3f} м)")
            return

        self.last_accepted = distance
        self.window.append(distance)
        moving_avg = sum(self.window) / len(self.window)

        print(
            f"Дистанция: {distance:.3f} м | "
            f"Скользящее среднее ({len(self.window)}/{self.window.maxlen}): {moving_avg:.3f} м"
        )


def choose_mode() -> int:
    """Спрашивает у пользователя, какой режим запустить."""
    print("Выберите режим работы:")
    print("  1 - Мин / макс / среднее за последние 100 измерений")
    print("  2 - Обнаружение объекта ближе 200 мм")
    print("  3 - Обнаружение объекта ближе 1.5 м")
    print("  4 - Фильтр выбросов + скользящее среднее")

    while True:
        choice = input("Введите номер режима (1-4): ").strip()
        if choice in ("1", "2", "3", "4"):
            return int(choice)
        print("Некорректный ввод, попробуйте снова.")


def make_mode_handler(mode: int):
    if mode == 1:
        print(f"Режим 1: мин/макс/среднее за последние {WINDOW_SIZE} измерений\n")
        return ModeStats(WINDOW_SIZE)
    elif mode == 2:
        print(f"Режим 2: обнаружение объекта ближе {THRESHOLD_MM_200} мм\n")
        return ModeThreshold(THRESHOLD_MM_200 / 1000.0, f"< {THRESHOLD_MM_200} мм")
    elif mode == 3:
        print(f"Режим 3: обнаружение объекта ближе {THRESHOLD_M_1_5} м\n")
        return ModeThreshold(THRESHOLD_M_1_5, f"< {THRESHOLD_M_1_5} м")
    elif mode == 4:
        print(f"Режим 4: фильтр выбросов + скользящее среднее (окно {WINDOW_SIZE})\n")
        return ModeFilteredAverage(WINDOW_SIZE, OUTLIER_JUMP_M)
    else:
        raise ValueError(f"Неизвестный режим={mode}. Допустимые значения: 1, 2, 3, 4")


# ==================== ОСНОВНОЙ ЦИКЛ ====================

def read_loop(uart: serial.Serial, handler) -> None:
    buffer = bytearray()
    try:
        while True:
            if uart.in_waiting > 0:
                buffer += uart.read(uart.in_waiting)
            else:
                time.sleep(0.005)
                continue

            while len(buffer) >= FRAME_LEN:
                if buffer[0] != FRAME_HEADER or buffer[1] != FUNCTION_MARK:
                    buffer.pop(0)
                    continue

                candidate = bytes(buffer[:FRAME_LEN])
                result = parse_frame(candidate)

                if result is not None:
                    handler.process(result)
                    del buffer[:FRAME_LEN]
                else:
                    buffer.pop(0)

    except KeyboardInterrupt:
        print("\nПрограмма остановлена пользователем (Ctrl+C)")


def main():
    uart = None
    try:
        mode = choose_mode()
        handler = make_mode_handler(mode)
        uart = open_uart(UART_PORT, BAUDRATE, TIMEOUT)
        read_loop(uart, handler)
    except serial.SerialException as e:
        print(f"Ошибка UART: {e}")
        print("Проверьте: включен ли UART, правильно ли подключены TX/RX, верна ли скорость")
    finally:
        if uart is not None and uart.is_open:
            uart.close()
            print("UART порт закрыт")


if __name__ == "__main__":
    main()
