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
"""

import serial
import time

# ==================== НАСТРОЙКИ ====================
UART_PORT = "/dev/serial0"
BAUDRATE = 921600      # заводское значение по умолчанию для этого датчика
TIMEOUT = 1
FRAME_LEN = 16
FRAME_HEADER = 0x57
FUNCTION_MARK = 0x00
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


def read_loop(uart: serial.Serial) -> None:
    buffer = bytearray()
    try:
        while True:
            if uart.in_waiting > 0:
                buffer += uart.read(uart.in_waiting)
            else:
                time.sleep(0.005)
                continue

            # Ищем в буфере валидный кадр: 0x57 0x00 ...
            while len(buffer) >= FRAME_LEN:
                # Синхронизация: ищем начало кадра
                if buffer[0] != FRAME_HEADER or buffer[1] != FUNCTION_MARK:
                    buffer.pop(0)  # сдвигаем на 1 байт и пробуем снова
                    continue

                candidate = bytes(buffer[:FRAME_LEN])
                result = parse_frame(candidate)

                if result is not None:
                    status = "OK" if result["valid"] else "невалидно"
                    print(
                        f"Дистанция: {result['distance_m']:.3f} м | "
                        f"Сигнал: {result['signal_strength']} | "
                        f"Точность: {result['precision_cm']} см | "
                        f"Статус: {status}"
                    )
                    del buffer[:FRAME_LEN]
                else:
                    # Контрольная сумма не сошлась — сдвигаемся на 1 байт
                    buffer.pop(0)

    except KeyboardInterrupt:
        print("\nПрограмма остановлена пользователем (Ctrl+C)")


def main():
    uart = None
    try:
        uart = open_uart(UART_PORT, BAUDRATE, TIMEOUT)
        read_loop(uart)
    except serial.SerialException as e:
        print(f"Ошибка UART: {e}")
        print("Проверьте: включен ли UART, правильно ли подключены TX/RX, верна ли скорость")
    finally:
        if uart is not None and uart.is_open:
            uart.close()
            print("UART порт закрыт")


if __name__ == "__main__":
    main()
