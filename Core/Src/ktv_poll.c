#include "ktv_poll.h"
#include "main.h"

#include <string.h>

/* === Параметры протокола КТВ === */
/* Длительность тика в мс (TIM2 = 2 мс). */
#define KTV_TICK_IN_MSEC 2U

/* Пауза между пакетами (в тиках). */
#define KTV_PAUSE_IN_TICKS 16U

/* Пауза перед началом первого КТВ (в тиках). */
#define KTV_START_PAUSE_IN_TICKS 20U

/* Длительность стартового импульса (в тиках). */
#define TICK_NUM_KTV_START 32U

/* Длительность логического "битового" участка и паузы КТВ (в тиках). */
#define TICK_NUM_KTV_BIT   16U
#define TICK_NUM_KTV_PAUSE KTV_PAUSE_IN_TICKS

/* Сдвиг до окна синхронизации (используется как ожидание перед чтением). */
#define SYNC_PULSE_OFFSET_IN_TICK 75U
#define SYNC_PULSE_WIDTH_IN_TICK  4U
#define TICK_NUM_TO_SYNC (SYNC_PULSE_OFFSET_IN_TICK + SYNC_PULSE_WIDTH_IN_TICK)

/* Интервал повторного опроса (мс). */
#define KTV_POLL_INTERVAL_MS 5000U

/* Длина одной "зоны" КТВ: пауза + два битовых участка + пауза. */
#define TICK_NUM_KTV (TICK_NUM_KTV_PAUSE + (TICK_NUM_KTV_BIT * 2U) + TICK_NUM_KTV_PAUSE)

/* Окно профиля: зона КТВ + дополнительная пауза (как в portable_code). */
#define TICK_NUM_KTV_AREA (TICK_NUM_KTV + TICK_NUM_KTV_PAUSE)

/* Общий размер битовой карты с запасом. */
#define KTV_BITMAP_SIZE ((KTV_SENSOR_COUNT * TICK_NUM_KTV) + TICK_NUM_KTV_START + 128U)

typedef struct
{
  uint8_t before;
  uint8_t online;
  uint8_t brk;
  uint8_t after;
} KtvResult;

typedef enum
{
  KTV_STATE_BOOT = 0,
  KTV_STATE_WAIT,
  KTV_STATE_ST_PULSE,
  KTV_STATE_SYNC,
  KTV_STATE_READ,
  KTV_STATE_END,
  KTV_STATE_NO_ACTIVE
} KtvState;

static volatile KtvState ktv_state = KTV_STATE_BOOT;
static volatile int32_t ktv_counter = 0;
static volatile int32_t ktv_wait_counter = 0;
static volatile uint32_t ktv_buff_idx = 0;

static uint64_t ktv_bitmap[KTV_BITMAP_SIZE / 64U + 1U];
static KtvResult ktv_results[KTV_SENSOR_COUNT];
static uint8_t ktv_status[KTV_STATUS_SIZE];

static void Ktv_ClearResults(void)
{
  memset(ktv_results, 0, sizeof(ktv_results));
}

static void Ktv_ClearStatus(void)
{
  memset(ktv_status, 0, sizeof(ktv_status));
}

static uint64_t Ktv_GetCurrProfile(int32_t bitmap_idx)
{
  /* Вытаскиваем окно длиной TICK_NUM_KTV_AREA из битовой карты. */
  const uint64_t bit = 1ULL;
  uint64_t mask = (bit << TICK_NUM_KTV_AREA);
  uint64_t prof;

  mask -= 1ULL;

  if (bitmap_idx < 0)
  {
    prof = (ktv_bitmap[0] << (uint32_t)(-bitmap_idx));
    prof &= mask;
    return prof;
  }

  const uint32_t pos = (uint32_t)bitmap_idx / 64U;
  const uint32_t offset = (uint32_t)bitmap_idx % 64U;
  const uint32_t rem = 64U - offset;
  const uint32_t rem1 = TICK_NUM_KTV_AREA - rem;
  uint64_t mask1;

  mask = ((bit << rem) - 1ULL);
  prof = (ktv_bitmap[pos] >> offset) & mask;

  if (rem1 > 0U)
  {
    mask1 = ((bit << rem1) - 1ULL);
    uint64_t prof1 = ktv_bitmap[pos + 1U] & mask1;
    prof1 <<= rem;
    prof |= prof1;
  }

  return prof;
}

static void Ktv_UpdateStatus(uint8_t idx, uint8_t flags)
{
  const uint32_t base = (uint32_t)idx * KTV_ITEMS_PER_SENSOR;

  /* 0 элемент - связь (1/0). */
  ktv_status[base] = (flags & 0x01U) ? 1U : 0U;

  /* 1 элемент - состояние: бит0=сработка, бит1=ошибка. */
  ktv_status[base + 1U] = flags & (KTV_STATE_TRIGGERED | KTV_STATE_ERROR);
}

static void Ktv_ProcessProfile(void)
{
  uint64_t start = ktv_bitmap[0];
  uint32_t count = 0U;
  uint32_t base_start = 4U;
  int32_t start_idx = -1;

  /* Поиск стартовой последовательности. */
  while (count < 3U)
  {
    start_idx = -1;
    for (uint32_t i = base_start; i < 48U; ++i)
    {
      if ((start & (1ULL << i)) != 0ULL)
      {
        start_idx = (int32_t)i;
        break;
      }
    }

    if (start_idx < 0)
    {
      /* Нет стартовой последовательности - считаем что КТВ неактивны. */
      Ktv_ClearResults();
      Ktv_ClearStatus();
      ktv_state = KTV_STATE_NO_ACTIVE;
      return;
    }

    count = 0U;
    for (uint32_t i = (uint32_t)start_idx; i < 32U; ++i)
    {
      if ((start & (1ULL << i)) != 0ULL)
      {
        ++count;
      }
      else
      {
        break;
      }
    }

    base_start = (uint32_t)start_idx + count;
  }

  if ((count >= 28U) && (base_start == 32U))
  {
    /* В начале одни "1" - активных КТВ нет. */
    Ktv_ClearResults();
    Ktv_ClearStatus();
    ktv_state = KTV_STATE_NO_ACTIVE;
    return;
  }

  const uint32_t profile_start = (uint32_t)start_idx + KTV_START_PAUSE_IN_TICKS;

  for (uint8_t i = 0U; i < KTV_SENSOR_COUNT; ++i)
  {
    uint8_t flags = 0U;
    uint64_t profile = Ktv_GetCurrProfile((int32_t)(TICK_NUM_KTV * i) + (int32_t)profile_start);

    /* Подсчёт Before/Online/Break/After. */
    uint32_t idx = TICK_NUM_KTV_PAUSE;
    uint32_t c = 0U;

    for (int32_t k = (int32_t)idx - 1; k >= 0; --k)
    {
      if ((profile & (1ULL << (uint32_t)k)) != 0ULL)
      {
        ++c;
      }
      else
      {
        break;
      }
    }
    ktv_results[i].before = (uint8_t)c;

    c = 0U;
    for (; idx < (TICK_NUM_KTV_PAUSE + TICK_NUM_KTV_BIT); ++idx)
    {
      if ((profile & (1ULL << idx)) != 0ULL)
      {
        ++c;
      }
    }
    ktv_results[i].online = (uint8_t)c;

    c = 0U;
    for (; idx < (TICK_NUM_KTV_PAUSE + (TICK_NUM_KTV_BIT * 2U)); ++idx)
    {
      if ((profile & (1ULL << idx)) != 0ULL)
      {
        ++c;
      }
    }
    ktv_results[i].brk = (uint8_t)c;

    c = 0U;
    const uint32_t border = TICK_NUM_KTV_PAUSE + (TICK_NUM_KTV_BIT * 2U) + TICK_NUM_KTV_PAUSE;
    for (; idx < border; ++idx)
    {
      if ((profile & (1ULL << idx)) != 0ULL)
      {
        ++c;
      }
      else
      {
        break;
      }
    }
    ktv_results[i].after = (uint8_t)c;

    /* Обработка ошибок по Before/After. */
    int32_t length = (int32_t)ktv_results[i].online + (int32_t)ktv_results[i].brk;
    if (ktv_results[i].before == 0U)
    {
      length += ktv_results[i].after;
      if (ktv_results[i].after >= TICK_NUM_KTV_PAUSE)
      {
        flags |= KTV_STATE_ERROR;
      }
    }
    else if (ktv_results[i].after == 0U)
    {
      length += ktv_results[i].before;
      if (ktv_results[i].before >= TICK_NUM_KTV_PAUSE)
      {
        flags |= KTV_STATE_ERROR;
      }
    }

    /* Состояние "связь" и "сработка". */
    if (length >= (int32_t)(TICK_NUM_KTV_BIT - 2U))
    {
      flags |= 0x01U; /* связь (online) */
    }
    if (length >= (int32_t)((TICK_NUM_KTV_BIT - 2U) * 2U))
    {
      flags |= KTV_STATE_TRIGGERED;
    }

    Ktv_UpdateStatus(i, flags);
  }
}

static void Ktv_StartPoll(void)
{
  /* Стартовый импульс на PWR.KTV.BUF: активный "0". */
  HAL_GPIO_WritePin(PWR_KTV_BUF_GPIO_Port, PWR_KTV_BUF_Pin, GPIO_PIN_RESET);
  ktv_counter = (int32_t)TICK_NUM_KTV_START;
  ktv_state = KTV_STATE_ST_PULSE;
}

void Ktv_Init(void)
{
  memset(ktv_bitmap, 0, sizeof(ktv_bitmap));
  Ktv_ClearResults();
  Ktv_ClearStatus();

  /* По умолчанию питание буфера отпущено (лог.1). */
  HAL_GPIO_WritePin(PWR_KTV_BUF_GPIO_Port, PWR_KTV_BUF_Pin, GPIO_PIN_SET);

  /* Пауза после включения питания. */
  ktv_counter = (int32_t)(KTV_POLL_INTERVAL_MS / KTV_TICK_IN_MSEC);
  ktv_wait_counter = ktv_counter;
  ktv_state = KTV_STATE_BOOT;
}

void Ktv_Process(void)
{
  if (ktv_state == KTV_STATE_END)
  {
    Ktv_ProcessProfile();

    /* Пауза перед следующим опросом. */
    ktv_wait_counter = (int32_t)(KTV_POLL_INTERVAL_MS / KTV_TICK_IN_MSEC);
    ktv_state = KTV_STATE_WAIT;
  }

  if (ktv_state == KTV_STATE_WAIT && ktv_wait_counter <= 0)
  {
    Ktv_StartPoll();
  }
}

void Ktv_TickISR(void)
{
  const GPIO_PinState pin_state = HAL_GPIO_ReadPin(KTV_ADR_GPIO_Port, KTV_ADR_Pin);
  const uint8_t pin = (pin_state == GPIO_PIN_SET) ? 0U : 1U;

  switch (ktv_state)
  {
    case KTV_STATE_BOOT:
      if (--ktv_counter <= 0)
      {
        ktv_state = KTV_STATE_WAIT;
        ktv_wait_counter = 0;
      }
      break;
    case KTV_STATE_WAIT:
      if (ktv_wait_counter > 0)
      {
        --ktv_wait_counter;
      }
      break;
    case KTV_STATE_ST_PULSE:
      if (--ktv_counter <= 0)
      {
        HAL_GPIO_WritePin(PWR_KTV_BUF_GPIO_Port, PWR_KTV_BUF_Pin, GPIO_PIN_SET);
        ktv_counter = (int32_t)TICK_NUM_TO_SYNC;
        ktv_state = KTV_STATE_SYNC;
      }
      break;
    case KTV_STATE_SYNC:
      if (--ktv_counter <= 0)
      {
        ktv_buff_idx = 0U;
        ktv_state = KTV_STATE_READ;
      }
      break;
    case KTV_STATE_READ:
    {
      const uint32_t pos = ktv_buff_idx / 64U;
      const uint32_t idx = ktv_buff_idx % 64U;
      const uint64_t bit = 1ULL;

      if (pin != 0U)
      {
        ktv_bitmap[pos] |= (bit << idx);
      }
      else
      {
        ktv_bitmap[pos] &= ~(bit << idx);
      }

      ++ktv_buff_idx;
      if (ktv_buff_idx >= KTV_BITMAP_SIZE)
      {
        ktv_state = KTV_STATE_END;
      }
      break;
    }
    case KTV_STATE_END:
    case KTV_STATE_NO_ACTIVE:
    default:
      break;
  }
}

const uint8_t *Ktv_GetStatusBuffer(void)
{
  return ktv_status;
}
