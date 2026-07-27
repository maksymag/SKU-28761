/*
 * tof_mini_reader.c
 *
 * Чтение данных с датчика Waveshare TOF Laser Range Sensor Mini (SKU 28761)
 * через UART на Raspberry Pi. Датчик отправляет бинарные пакеты по 16 байт
 * в формате протокола NLink_TOFSense_Frame0 (не текст!).
 *
 * Формат пакета (little-endian):
 *   [0]     Frame Header      = 0x57
 *   [1]     Function Mark     = 0x00
 *   [2]     reserved          = 0xff
 *   [3]     id
 *   [4:8]   System_time (мс), uint32
 *   [8:11]  dis*1000 (мм), uint24 (3 байта)
 *   [11]    dis_status  (0 = невалидно, 1 = валидно)
 *   [12:14] signal_strength, uint16
 *   [14]    range_precision (см)
 *   [15]    Sum Check (контрольная сумма)
 *
 * РЕЖИМЫ РАБОТЫ (выбираются при запуске):
 *   1 - Мин / макс / среднее за последние 100 измерений
 *   2 - Обнаружение объекта ближе 200 мм
 *   3 - Обнаружение объекта ближе 1.5 м
 *   4 - Фильтр выбросов + скользящее среднее
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <math.h>

#define UART_PORT       "/dev/serial0"
#define BAUDRATE        B921600
#define FRAME_LEN       16
#define FRAME_HEADER    0x57
#define FUNCTION_MARK   0x00

#define WINDOW_SIZE       100      /* для режима 1 и 4 */
#define THRESHOLD_MM_200  200.0    /* для режима 2 (мм) */
#define THRESHOLD_M_1_5   1.5      /* для режима 3 (м) */
#define OUTLIER_JUMP_M    0.5      /* для режима 4 (м) */

typedef struct {
    uint8_t  id;
    uint32_t time_ms;
    double   distance_m;
    int      valid;
    uint16_t signal_strength;
    uint8_t  precision_cm;
} TofResult;

/* ==================== Открытие UART ==================== */

int open_uart(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Ошибка открытия порта %s: %s\n", port, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "Ошибка tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag |= (CLOCAL | CREAD);   /* включить приём, игнорировать статус линии */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                /* 8 бит данных */
    tty.c_cflag &= ~PARENB;            /* без чётности */
    tty.c_cflag &= ~CSTOPB;            /* 1 стоп-бит */
    tty.c_cflag &= ~CRTSCTS;           /* без аппаратного управления потоком */

    tty.c_lflag = 0;                   /* raw режим, без канонической обработки */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICRNL | INLCR);
    tty.c_oflag = 0;

    tty.c_cc[VMIN]  = 0;    /* неблокирующее чтение */
    tty.c_cc[VTIME] = 1;    /* таймаут 0.1 сек */

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Ошибка tcsetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    printf("UART порт %s открыт на скорости 921600 бод\n", port);
    return fd;
}

/* ==================== Парсинг кадра ==================== */

int checksum_ok(const uint8_t *frame) {
    uint32_t sum = 0;
    for (int i = 0; i < FRAME_LEN - 1; i++) {
        sum += frame[i];
    }
    return (sum & 0xFF) == frame[FRAME_LEN - 1];
}

int parse_frame(const uint8_t *frame, TofResult *out) {
    if (frame[0] != FRAME_HEADER || frame[1] != FUNCTION_MARK) {
        return 0;
    }
    if (!checksum_ok(frame)) {
        return 0;
    }

    out->id = frame[3];
    out->time_ms = (uint32_t)frame[4] | ((uint32_t)frame[5] << 8) |
                   ((uint32_t)frame[6] << 16) | ((uint32_t)frame[7] << 24);

    uint32_t dis_raw = (uint32_t)frame[8] | ((uint32_t)frame[9] << 8) |
                        ((uint32_t)frame[10] << 16);
    out->distance_m = dis_raw / 1000.0;

    out->valid = (frame[11] == 1);
    out->signal_strength = (uint16_t)frame[12] | ((uint16_t)frame[13] << 8);
    out->precision_cm = frame[14];

    return 1;
}

/* ==================== Режимы работы ==================== */

typedef struct {
    double values[WINDOW_SIZE];
    int count;      /* сколько значений реально накоплено (<= WINDOW_SIZE) */
    int next_idx;   /* куда писать следующее значение (кольцевой буфер) */
} SlidingWindow;

void window_init(SlidingWindow *w) {
    w->count = 0;
    w->next_idx = 0;
}

void window_push(SlidingWindow *w, double value) {
    w->values[w->next_idx] = value;
    w->next_idx = (w->next_idx + 1) % WINDOW_SIZE;
    if (w->count < WINDOW_SIZE) {
        w->count++;
    }
}

void window_stats(const SlidingWindow *w, double *out_min, double *out_max, double *out_avg) {
    double mn = w->values[0], mx = w->values[0], sum = 0.0;
    for (int i = 0; i < w->count; i++) {
        double v = w->values[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    *out_min = mn;
    *out_max = mx;
    *out_avg = sum / w->count;
}

/* ---- Режим 1: мин/макс/среднее ---- */
void mode1_process(SlidingWindow *w, const TofResult *r) {
    if (!r->valid) return;
    window_push(w, r->distance_m);

    double mn, mx, avg;
    window_stats(w, &mn, &mx, &avg);

    printf("Дистанция: %.3f м | Окно: %d/%d | Мин: %.3f м | Макс: %.3f м | Среднее: %.3f м\n",
           r->distance_m, w->count, WINDOW_SIZE, mn, mx, avg);
}

/* ---- Режимы 2 и 3: обнаружение объекта ближе порога ---- */
typedef struct {
    double threshold_m;
    const char *label;
    int was_close;
} ThresholdState;

void threshold_process(ThresholdState *s, const TofResult *r) {
    if (!r->valid) return;

    int is_close = r->distance_m < s->threshold_m;

    if (is_close && !s->was_close) {
        printf("!! ОБЪЕКТ ОБНАРУЖЕН (%s)! Дистанция: %.3f м\n", s->label, r->distance_m);
    } else if (!is_close && s->was_close) {
        printf("Объект вышел за пределы %s. Дистанция: %.3f м\n", s->label, r->distance_m);
    } else {
        printf("Дистанция: %.3f м | Близко (%s): %s\n",
               r->distance_m, s->label, is_close ? "да" : "нет");
    }

    s->was_close = is_close;
}

/* ---- Режим 4: фильтр выбросов + скользящее среднее ---- */
typedef struct {
    SlidingWindow window;
    double max_jump_m;
    double last_accepted;
    int has_last;
} FilteredAvgState;

void filtered_avg_init(FilteredAvgState *s, double max_jump_m) {
    window_init(&s->window);
    s->max_jump_m = max_jump_m;
    s->has_last = 0;
}

void filtered_avg_process(FilteredAvgState *s, const TofResult *r) {
    if (!r->valid) return;

    double distance = r->distance_m;

    if (s->has_last && fabs(distance - s->last_accepted) > s->max_jump_m) {
        printf("Выброс отфильтрован: %.3f м (пред. значение: %.3f м)\n",
               distance, s->last_accepted);
        return;
    }

    s->last_accepted = distance;
    s->has_last = 1;
    window_push(&s->window, distance);

    double mn, mx, avg;
    window_stats(&s->window, &mn, &mx, &avg);

    printf("Дистанция: %.3f м | Скользящее среднее (%d/%d): %.3f м\n",
           distance, s->window.count, WINDOW_SIZE, avg);
}

/* ==================== Выбор режима ==================== */

int choose_mode(void) {
    int choice;
    char line[16];

    printf("Выберите режим работы:\n");
    printf("  1 - Мин / макс / среднее за последние %d измерений\n", WINDOW_SIZE);
    printf("  2 - Обнаружение объекта ближе %.0f мм\n", THRESHOLD_MM_200);
    printf("  3 - Обнаружение объекта ближе %.1f м\n", THRESHOLD_M_1_5);
    printf("  4 - Фильтр выбросов + скользящее среднее\n");

    while (1) {
        printf("Введите номер режима (1-4): ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) {
            continue;
        }
        choice = atoi(line);
        if (choice >= 1 && choice <= 4) {
            return choice;
        }
        printf("Некорректный ввод, попробуйте снова.\n");
    }
}

/* ==================== Основной цикл ==================== */

int main(void) {
    int fd = open_uart(UART_PORT);
    if (fd < 0) {
        fprintf(stderr, "Проверьте: включен ли UART, правильно ли подключены TX/RX, верна ли скорость\n");
        return 1;
    }

    int mode = choose_mode();

    SlidingWindow window1;
    ThresholdState threshold_state = {0};
    FilteredAvgState filtered_state;

    switch (mode) {
        case 1:
            window_init(&window1);
            printf("Режим 1: мин/макс/среднее за последние %d измерений\n\n", WINDOW_SIZE);
            break;
        case 2:
            threshold_state.threshold_m = THRESHOLD_MM_200 / 1000.0;
            threshold_state.label = "< 200 мм";
            printf("Режим 2: обнаружение объекта ближе %.0f мм\n\n", THRESHOLD_MM_200);
            break;
        case 3:
            threshold_state.threshold_m = THRESHOLD_M_1_5;
            threshold_state.label = "< 1.5 м";
            printf("Режим 3: обнаружение объекта ближе %.1f м\n\n", THRESHOLD_M_1_5);
            break;
        case 4:
            filtered_avg_init(&filtered_state, OUTLIER_JUMP_M);
            printf("Режим 4: фильтр выбросов + скользящее среднее (окно %d)\n\n", WINDOW_SIZE);
            break;
    }

    uint8_t buffer[4096];
    int buf_len = 0;

    while (1) {
        uint8_t chunk[256];
        ssize_t n = read(fd, chunk, sizeof(chunk));

        if (n > 0) {
            if (buf_len + n > (int)sizeof(buffer)) {
                /* буфер переполнен - сбрасываем, такое не должно происходить в норме */
                buf_len = 0;
            }
            memcpy(buffer + buf_len, chunk, n);
            buf_len += n;
        }

        /* Ищем и разбираем кадры в буфере */
        int i = 0;
        while (buf_len - i >= FRAME_LEN) {
            if (buffer[i] != FRAME_HEADER || buffer[i + 1] != FUNCTION_MARK) {
                i++;
                continue;
            }

            TofResult result;
            if (parse_frame(buffer + i, &result)) {
                switch (mode) {
                    case 1: mode1_process(&window1, &result); break;
                    case 2:
                    case 3: threshold_process(&threshold_state, &result); break;
                    case 4: filtered_avg_process(&filtered_state, &result); break;
                }
                i += FRAME_LEN;
            } else {
                i++;
            }
        }

        /* сдвигаем оставшиеся необработанные байты в начало буфера */
        memmove(buffer, buffer + i, buf_len - i);
        buf_len -= i;

        if (n <= 0) {
            usleep(5000); /* 5 мс, чтобы не грузить CPU впустую */
        }
    }

    close(fd);
    return 0;
}
