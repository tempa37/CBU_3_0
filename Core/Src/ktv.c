#include "ktv.h"

#include "main.h"
#include <string.h>

typedef enum
{
  ksNone,
  ksStart,    /* ожидание старта после включения */
  ksStarted,  /* готов к началу опроса */
  ksWait,     /* пауза между опросами */
  ksStPulse,  /* стартовый импульс */
  ksSync,     /* ожидание синхроимпульса */
  ksRead,     /* чтение последовательности */
  ksEnd,      /* чтение завершено */
  ksNoActive  /* на линии нет активности */
} KtvState;

typedef struct
{
  uint8_t before;
  uint8_t online;
  uint8_t brk;
  uint8_t after;
} KtvSegment;

typedef struct
{
  KtvState state;
  int32_t counter;
  int64_t buff_idx;
  uint64_t bitmap[KTV_BITMAP_SIZE / 64U + 1U];
  uint8_t flags[KTV_NUM_MAX + 1U];
  KtvSegment segments[KTV_NUM_MAX + 1U];
} KtvContext;

static KtvContext g_ktv;
static KtvResultBuffer g_result;

static void Ktv_Start(void);
static uint64_t Ktv_GetProfile(int32_t bitmap_idx);
static void Ktv_ClearResults(void);
static bool Ktv_ProcessProfile(void);

void Ktv_Init(void)
{
  memset(&g_ktv, 0, sizeof(g_ktv));
  memset(&g_result, 0, sizeof(g_result));

  /* По умолчанию питание КТВ включено. */
  HAL_GPIO_WritePin(PWR_KTV_BUF_GPIO_Port, PWR_KTV_BUF_Pin, GPIO_PIN_SET);

  g_ktv.counter = (int32_t)(KTV_BOOTUP_INTERVAL / KTV_TICK_IN_MSEC);
  g_ktv.state = ksStart;
}

static void Ktv_Start(void)
{
  /* Стартовый импульс: тянем PWR.KTV.BUF вниз. */
  HAL_GPIO_WritePin(PWR_KTV_BUF_GPIO_Port, PWR_KTV_BUF_Pin, GPIO_PIN_RESET);

  g_ktv.counter = (int32_t)TICK_NUM_KTV_START;
  g_ktv.buff_idx = 0;
  memset(g_ktv.bitmap, 0, sizeof(g_ktv.bitmap));
  g_ktv.state = ksStPulse;
}

const KtvResultBuffer *Ktv_GetResultBuffer(void)
{
  return &g_result;
}

uint8_t Ktv_GetFlags(uint8_t ktv_index)
{
  if ((ktv_index == 0U) || (ktv_index > KTV_NUM_MAX))
  {
    return 0U;
  }

  return g_ktv.flags[ktv_index];
}

void Ktv_TickISR(void)
{
  /* Линия активна в логическом 0, поэтому инвертируем. */
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(KTV_ADR_GPIO_Port, KTV_ADR_Pin);
  uint8_t sample = (pin_state == GPIO_PIN_SET) ? 0U : 1U;

  switch (g_ktv.state)
  {
    case ksStart:
      if (--g_ktv.counter <= 0)
      {
        g_ktv.state = ksStarted;
      }
      break;
    case ksWait:
      if (--g_ktv.counter <= 0)
      {
        g_ktv.state = ksStarted;
      }
      break;
    case ksStPulse:
      if (--g_ktv.counter <= 0)
      {
        /* Возвращаем питание и начинаем чтение профиля. */
        HAL_GPIO_WritePin(PWR_KTV_BUF_GPIO_Port, PWR_KTV_BUF_Pin, GPIO_PIN_SET);
        g_ktv.counter = (int32_t)TICK_NUM_TO_SYNC;
        g_ktv.state = ksSync;
      }
      break;
    case ksSync:
      if (--g_ktv.counter <= 0)
      {
        g_ktv.state = ksRead;
      }
      break;
    case ksRead:
    {
      int32_t pos = (int32_t)(g_ktv.buff_idx / 64);
      int32_t idx = (int32_t)(g_ktv.buff_idx % 64);
      uint64_t bit = 1ULL;

      if (sample != 0U)
      {
        g_ktv.bitmap[pos] |= (bit << idx);
      }
      else
      {
        g_ktv.bitmap[pos] &= ~(bit << idx);
      }

      ++g_ktv.buff_idx;
      if (g_ktv.buff_idx >= (int64_t)KTV_BITMAP_SIZE)
      {
        g_ktv.state = ksEnd;
      }
      break;
    }
    default:
      break;
  }
}

void Ktv_Process(void)
{
  if (g_ktv.state == ksStarted)
  {
    Ktv_Start();
    return;
  }

  if (g_ktv.state == ksEnd)
  {
    (void)Ktv_ProcessProfile();
    g_ktv.state = ksWait;
    g_ktv.counter = (int32_t)(KTV_TEST_INTERVAL / KTV_TICK_IN_MSEC);
  }
  else if (g_ktv.state == ksNoActive)
  {
    Ktv_ClearResults();
    g_ktv.state = ksWait;
    g_ktv.counter = (int32_t)(KTV_TEST_INTERVAL / KTV_TICK_IN_MSEC);
  }
}

static uint64_t Ktv_GetProfile(int32_t bitmap_idx)
{
  /* Получаем окно в TICK_NUM_KTV_AREA тиков (КТВ + пауза). */
  const int32_t area_ticks = (int32_t)(TICK_NUM_KTV + TICK_NUM_KTV_PAUSE);
  uint64_t bit = 1ULL;
  uint64_t mask = (bit << area_ticks) - 1ULL;

  if (bitmap_idx < 0)
  {
    uint64_t prof = (g_ktv.bitmap[0] << (uint32_t)(-bitmap_idx));
    return prof & mask;
  }

  int32_t pos = bitmap_idx / 64;
  uint32_t offset = (uint32_t)(bitmap_idx % 64);
  uint32_t rem = 64U - offset;
  int32_t rem1 = (int32_t)area_ticks - (int32_t)rem;
  uint64_t prof = (g_ktv.bitmap[pos] >> offset) & ((bit << rem) - 1ULL);

  if (rem1 > 0)
  {
    uint64_t prof1 = g_ktv.bitmap[pos + 1];
    prof1 &= ((bit << (uint32_t)rem1) - 1ULL);
    prof1 <<= rem;
    prof |= prof1;
  }

  return prof & mask;
}

static void Ktv_ClearResults(void)
{
  memset(g_ktv.flags, 0, sizeof(g_ktv.flags));
  memset(g_ktv.segments, 0, sizeof(g_ktv.segments));
  memset(&g_result, 0, sizeof(g_result));
}

static bool Ktv_ProcessProfile(void)
{
  uint64_t start_word = g_ktv.bitmap[0];
  uint32_t count = 0U;
  uint32_t base_start = 4U;
  int32_t start_idx = -1;

  /* Ищем стартовую последовательность: минимум 3 подряд единицы. */
  while (count < 3U)
  {
    start_idx = -1;
    for (uint64_t i = base_start; i < 48U; ++i)
    {
      if ((start_word & (1ULL << i)) != 0U)
      {
        start_idx = (int32_t)i;
        break;
      }
    }

    if (start_idx < 0)
    {
      Ktv_ClearResults();
      g_ktv.state = ksNoActive;
      return true;
    }

    count = 0U;
    for (int32_t i = start_idx; i < 32; ++i)
    {
      if ((start_word & (1ULL << (uint32_t)i)) != 0U)
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
    /* Линия постоянно в "1" -> КТВ не активны. */
    Ktv_ClearResults();
    g_ktv.state = ksNoActive;
    return true;
  }

  uint32_t profile_start = (uint32_t)start_idx + KTV_START_PAUSE_IN_TICKS;

  for (uint32_t i = 0; i <= KTV_NUM_MAX; ++i)
  {
    uint64_t prof = Ktv_GetProfile((int32_t)(TICK_NUM_KTV * i) + (int32_t)profile_start);
    uint32_t idx = TICK_NUM_KTV_PAUSE;
    uint8_t value = 0U;
    uint32_t local_count = 0U;

    /* Считаем Before (единицы перед Online). */
    for (int32_t k = (int32_t)idx - 1; k >= 0; --k)
    {
      if ((prof & (1ULL << (uint32_t)k)) != 0U)
      {
        ++local_count;
      }
      else
      {
        break;
      }
    }
    g_ktv.segments[i].before = (uint8_t)local_count;

    /* Online. */
    local_count = 0U;
    for (; idx < (TICK_NUM_KTV_PAUSE + TICK_NUM_KTV_BIT); ++idx)
    {
      if ((prof & (1ULL << idx)) != 0U)
      {
        ++local_count;
      }
    }
    g_ktv.segments[i].online = (uint8_t)local_count;

    /* Break. */
    local_count = 0U;
    for (; idx < (TICK_NUM_KTV_PAUSE + (TICK_NUM_KTV_BIT * 2U)); ++idx)
    {
      if ((prof & (1ULL << idx)) != 0U)
      {
        ++local_count;
      }
    }
    g_ktv.segments[i].brk = (uint8_t)local_count;

    /* After (единицы после Break). */
    local_count = 0U;
    uint32_t border = (TICK_NUM_KTV_PAUSE + (TICK_NUM_KTV_BIT * 2U) + TICK_NUM_KTV_PAUSE);
    for (; idx < border; ++idx)
    {
      if ((prof & (1ULL << idx)) != 0U)
      {
        ++local_count;
      }
      else
      {
        break;
      }
    }
    g_ktv.segments[i].after = (uint8_t)local_count;

    uint32_t length = g_ktv.segments[i].online + g_ktv.segments[i].brk;
    if (g_ktv.segments[i].before == 0U)
    {
      length += g_ktv.segments[i].after;
      if (g_ktv.segments[i].after >= TICK_NUM_KTV_PAUSE)
      {
        value |= KTV_ERROR;
      }
    }
    else if (g_ktv.segments[i].after == 0U)
    {
      length += g_ktv.segments[i].before;
      if (g_ktv.segments[i].before >= TICK_NUM_KTV_PAUSE)
      {
        value |= KTV_ERROR;
      }
    }

    if (length >= (TICK_NUM_KTV_BIT - 2U))
    {
      value |= KTV_ONLINE;
    }
    if (length >= ((TICK_NUM_KTV_BIT - 2U) * 2U))
    {
      value |= KTV_TRIGGERED;
    }

    g_ktv.flags[i] = value;
  }

  /* Перекладываем в буфер 0..99 (КТВ1..КТВ50). */
  for (uint32_t i = 1; i <= KTV_NUM_MAX; ++i)
  {
    uint32_t offset = (i - 1U) * 2U;
    g_result.items[offset] = (g_ktv.flags[i] & KTV_ONLINE) ? 1U : 0U;
    g_result.items[offset + 1U] = (g_ktv.flags[i] & KTV_TRIGGERED) ? 1U : 0U;
  }

  return true;
}
