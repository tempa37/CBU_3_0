#ifndef KTV_H
#define KTV_H

#include <stdbool.h>
#include <stdint.h>

/* Количество КТВ в системе. */
#define KTV_NUM_MAX (50U)

/* 0 элемент = КТВ1 связь, 1 элемент = КТВ1 состояние, далее для 2..50. */
#define KTV_RESULT_ITEMS (KTV_NUM_MAX * 2U)

/* Флаги состояния КТВ. */
#define KTV_ONLINE    (0x01U)
#define KTV_TRIGGERED (0x02U)
#define KTV_ERROR     (0x04U)

/* Параметры протокола (перенесены из portable_code/task_inp_def.h). */
#define KTV_TICK_IN_MSEC      (2U)
#define TICK_NUM_PER_QUANT    (2U)
#define QUANT_NUM_KTV_BIT     (4U)
#define TICK_NUM_KTV_BIT      (QUANT_NUM_KTV_BIT * TICK_NUM_PER_QUANT)   
#define QUANT_NUM_KTV_PAUSE   (8U)
#define TICK_NUM_KTV_PAUSE    (QUANT_NUM_KTV_PAUSE * TICK_NUM_PER_QUANT) 
#define QUANT_NUM_KTV         ((QUANT_NUM_KTV_BIT * 2U) + QUANT_NUM_KTV_PAUSE)
#define TICK_NUM_KTV          (QUANT_NUM_KTV * TICK_NUM_PER_QUANT)     
#define QUANT_NUM_KTV_START   (16U)
#define TICK_NUM_KTV_START    (QUANT_NUM_KTV_START * TICK_NUM_PER_QUANT) 

#define KTV_PAUSE_IN_TICKS      (16U)
#define KTV_START_PAUSE_IN_TICKS (20U)
#define KTV_BOOTUP_INTERVAL     (5000U)
#define KTV_TEST_INTERVAL       (5000U)

/* Задержка до чтения профиля после стартового импульса (синхроимпульс). */
#define SYNC_PULSE_OFFSET_IN_TICK  (75U)
#define SYNC_PULSE_WIDTH_IN_TICK   (4U)
#define TICK_NUM_TO_SYNC (SYNC_PULSE_OFFSET_IN_TICK - KTV_PAUSE_IN_TICKS)

/* Размер буфера для записи всей посылки КТВ. */
#define KTV_BITMAP_SIZE (((KTV_NUM_MAX + 1U) * TICK_NUM_KTV) + TICK_NUM_KTV_START + 128U)

typedef struct
{
  uint8_t items[KTV_RESULT_ITEMS];
} KtvResultBuffer;

void Ktv_Init(void);
void Ktv_TickISR(void);
void Ktv_Process(void);
const KtvResultBuffer *Ktv_GetResultBuffer(void);
uint8_t Ktv_GetFlags(uint8_t ktv_index);

#endif /* KTV_H */
