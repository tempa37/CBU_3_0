#ifndef KTV_POLL_H
#define KTV_POLL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Количество КТВ в системе. */
#define KTV_SENSOR_COUNT 50U

/* В буфере на каждый КТВ: 0 = связь, 1 = состояние. */
#define KTV_ITEMS_PER_SENSOR 2U
#define KTV_STATUS_SIZE (KTV_SENSOR_COUNT * KTV_ITEMS_PER_SENSOR)

/* Биты состояния КТВ (элемент "состояние"). */
#define KTV_STATE_TRIGGERED 0x01U
#define KTV_STATE_ERROR     0x02U

/* Инициализация опроса КТВ. */
void Ktv_Init(void);

/* Периодическая обработка (вызывать в main loop). */
void Ktv_Process(void);

/* Обработчик тиков 2 мс (вызывать из TIM2 ISR). */
void Ktv_TickISR(void);

/* Доступ к буферу состояния (100 элементов). */
const uint8_t *Ktv_GetStatusBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* KTV_POLL_H */
