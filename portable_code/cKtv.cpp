/*
 * cKtv.cpp
 *
 *  Created on: 09 сен 2022 г.
 *      Author: Сосновских А.А.
 *
 *      Блок КТВ
 */

#include "cKtv.h"
#include "tim.h"


#if (USE_KTV_BLOCK == 1)
// ======= Блок входов типа КТВ

//Считывание входов КТВ: выполняется по отдельному запросу.
#include "string.h"
#include "io_pin.h"


#define KTV_STATES_SIZE 10
typedef struct sKtvState {
  eKtvState KtvState;
  eKtvMsg   KtvMsg;
} tsKtvState;

struct sKtvStates {
  uint32_t    Count;
  tsKtvState  aKtvState[KTV_STATES_SIZE];
  void Init() {
    Count = 0;
  }
  void AddState(eKtvState iKtvState, eKtvMsg iKtvMsg) {
    if (Count < KTV_STATES_SIZE) {
      aKtvState[Count].KtvState = iKtvState;
      aKtvState[Count].KtvMsg = iKtvMsg;
      ++Count;
    }
  }
} gKtvStates;

//tsKtvBlk::

#include "tim.h"
void tsKtvBlk::Init(tsKtvBlkDesc * ipKtvBlkDesc) {
  pKtvBlkDesc = ipKtvBlkDesc;
  if (paPinDesc == NULL) {
    //Считывание из блока Ethernet
    return;
  }
  //Начальное включение и стартовое обновление блока КТВ
  memset((void *)Bitmap, 0, sizeof(Bitmap));
  Pin_Set(&paPinDesc[1], GPIO_PIN_SET);
  Counter = KTV_BOOTUP_INTERVAL / KTV_TICK_IN_MSEC;
  gKtvStates.Init();
  CheckTime = 0;
  //Пуск таймера, период = 0,5кв
  Start_IT_TIM14();
  State = ksStart;
}

#include "cTaskDef.h"
uint32_t gKtvStartCount = 0, gKtvStartEndCount = 0;

void tsKtvBlk::Start() { //Пуск процесса КТВ
#ifdef KTV_START_PULSE
  Pin_Set(&paPinDesc[1], GPIO_PIN_RESET);
#endif
  ++gKtvStartCount;
  StartTick = GetTickCount();
  Counter = TICK_NUM_KTV_START;
  State = ksStPulse;
}

//Получить профиль ответа КТВ в виде uint64_t
#define TICK_NUM_KTV_AREA (TICK_NUM_KTV + TICK_NUM_KTV_PAUSE)
uint64_t tsKtvBlk::GetCurrKtvProf(int iBitmapIdx) {
  // !!! Возможность работы с 4 тиками на квант
  //iBitmapIdx может быть < 0 (-3 .. -1)
  uint64_t cBit = 1;
  uint64_t cMask = (cBit << TICK_NUM_KTV_AREA);
  cMask -= 1;
  uint64_t cProf;
  if (iBitmapIdx < 0) {
    cProf = (Bitmap[0] << -iBitmapIdx);
    cProf &= cMask;
    return cProf;
  }
  int cPos = iBitmapIdx / 64;
  uint64_t cOffset = iBitmapIdx % 64;
  uint64_t cRem = 64 - cOffset;
  uint64_t cRem1 = TICK_NUM_KTV_AREA - cRem;
  uint64_t cMask1;
  cMask = ((cBit << cRem) - 1);
//  cMask = (cBit << cRem);
//  cMask -= 1;
  uint64_t cProf1 = Bitmap[cPos];
  cProf = (cProf1 >> cOffset) & cMask;
  if (cRem1 > 0) {
    cMask1 = ((cBit << cRem1) - 1);
    cProf1 = Bitmap[cPos + 1];
    cProf1 &= cMask1;
    cProf1 = cProf1 << cRem; 
    cProf |= cProf1;
  }
  return cProf;
}

//Обработка буфера КТВ
extern "C" bool IsKbNorm();

int8_t gKtvCountOnline = 0, gKtvCountBreak = 0, gKtvStartIdx = 0;
int8_t gStartPause = 0;
tsKtvRezult aKtvRezult[KTV_NUM_MAX + 1];

void tsKtvBlk::ClearKtvResult() {
  memset((uint8_t *)aKtvRezult, 0, sizeof(aKtvRezult));
}

bool tsKtvBlk::ProcessKtvProf() {
  tsKtvElem * cpKtvElem = aKtvElem;
  //Алгоритм обработки:
  //- Найти первую "1" = точка отсчёта
  uint64_t cStart = Bitmap[0];
  uint32_t cStartIdx;
  uint32_t cCount = 0, cBaseStart = 4;
  int32_t cStart1;
  while (cCount < 3) {
    cStart1 = -1;
    for (uint64_t i = cBaseStart; i < 48; ++i) {
      if ((cStart & (1 << i)) != 0) {
        cStart1 = i;
        break;
      }
    }
    
    if (cStart1 < 0) {
      cStart1 = 0;
      ClearKtvResult();
      State = ksNoActive;
      return true;
    }
    cCount = 0;
    for (int i = cStart1; i < 32; ++i) {
      if ((cStart & (1 << i)) != 0) {
        ++cCount;
      } else {
        if ((cStart & (1 << i)) != 0);
        break;
      }
    }
    cBaseStart = cStart1 + cCount;
  }
  
  if ((cCount >= 28) && (cBaseStart == 32)) {
    //Это означает разрыв адресной линии КТВ (В битовой карте все '1')
    ClearKtvResult();
    State = ksNoActive;
    IsTriggered = false;
    return true;
  }
  
  bool bChanged = false;
  //По стартовому импульсу - начало отсчёта ответов датчиков КТВ
  cStartIdx = cStart1 + KTV_START_PAUSE_IN_TICKS/* + 1*/;
  
  gKtvStartIdx = cStartIdx;
  for (int i = 0; i < 32; ++i) {
    if (cStart & (1 << i))
      ++cCount;
  }
  //Обработка профилей КТВ
  uint64_t cProf, cBit = 1;
  tsEnumValue  cValue;
  int cNumber = gApp.pAppDesc->KtvSensNum;
  Profile = 0;
  IsTriggered = false;
  for (int i = 0; i < (KTV_NUM_MAX + 1); ++i) { //По всем КТВ:
    //Пропуск неиспользуемого интервала КТВ
    if ((i > cNumber) && (i < KTV_DPP_POS)) {
      continue;
    }
    cValue = cpKtvElem[i].EnumValue;
    cpKtvElem[i].EnumValue.Enum = 0;
    cProf = GetCurrKtvProf((TICK_NUM_KTV * i) + cStartIdx);
    //Вычисление параметров профиля ответа датчика КТВ
    uint8_t cIdx = TICK_NUM_KTV_PAUSE;
    cCount = 0;
    for (int k = cIdx - 1; k >= 0; --k) {
      if (cProf & (cBit << k)) {
        ++cCount;
      } else { //Если первый же тик == 0 -> Before = cCount
        break;
      }
    }
    aKtvRezult[i].Before = cCount;
    cCount = 0;
    for ( ; cIdx < (TICK_NUM_KTV_PAUSE + TICK_NUM_KTV_BIT); ++cIdx) {
      if (cProf & (cBit << cIdx))
        ++cCount;
    }
    aKtvRezult[i].Online = cCount;
    // !!! Другой алгоритм вычисления параметров - по длительности импульса
//    if (cCount >= (TICK_NUM_KTV_BIT / 2 + 1)) {
//      //Есть сигнал "На связи"
//      cpKtvElem[i].EnumValue.Enum |= KTV_ONLINE;
//    }
    cCount = 0;
    for ( ; cIdx < (TICK_NUM_KTV_PAUSE + (TICK_NUM_KTV_BIT * 2)); ++cIdx) {
      if (cProf & (cBit << cIdx))
        ++cCount;
    }
    aKtvRezult[i].Break = cCount;
    // !!! Другой алгоритм вычисления параметров - по длительности импульса
//    if (cCount > (TICK_NUM_KTV_BIT / 2 + 1)) {
//      //Есть сигнал "Сработал"
//      cpKtvElem[i].EnumValue.Enum |= KTV_TRIGGERED;
//    }
    cCount = 0;
    int cBorder = (TICK_NUM_KTV_PAUSE + (TICK_NUM_KTV_BIT * 2) + (TICK_NUM_KTV_PAUSE));
    for ( ; cIdx < cBorder; ++cIdx) {
      if (cProf & (cBit << cIdx)) {
        ++cCount;
      } else { //Если первый же тик == 0 -> After = cCount
        break;
      }
    }
    aKtvRezult[i].After = cCount;
    
    int cLength = aKtvRezult[i].Online + aKtvRezult[i].Break;
    if (aKtvRezult[i].Before == 0) {
      cLength += aKtvRezult[i].After;
      if (aKtvRezult[i].After >= TICK_NUM_KTV_PAUSE) {
        cpKtvElem->EnumValue.Enum |= KTV_ERROR;
      }
    } else if (aKtvRezult[i].After == 0) {
      cLength += aKtvRezult[i].Before;
      if (aKtvRezult[i].Before >= TICK_NUM_KTV_PAUSE) {
        cpKtvElem->EnumValue.Enum |= KTV_ERROR;
      }
    }
    if (cLength >= (TICK_NUM_KTV_BIT - 2))
      cpKtvElem[i].EnumValue.Enum |= KTV_ONLINE;
    if (cLength >= ((TICK_NUM_KTV_BIT - 2) * 2)) {
      cpKtvElem[i].EnumValue.Enum |= KTV_TRIGGERED;
      uint64_t cV = 1;
      Profile |= (cV << i);
      IsTriggered = true;
    }
    // !!! Другой алгоритм вычисления параметров - по длительности импульса
//    //  Вычислить ошибку
//    if (!(cpKtvElem[i].EnumValue.Enum & KTV_ONLINE) && (cpKtvElem[i].EnumValue.Enum & KTV_TRIGGERED))
//      cpKtvElem[i].EnumValue.Enum |= KTV_ERROR;
//    else
//      cpKtvElem[i].EnumValue.Enum &= ~KTV_ERROR;
    cpKtvElem[i].Changed = (cpKtvElem[i].EnumValue.Enum != cValue.Enum);
    if (cpKtvElem[i].Changed)
      bChanged = true;
  }
  Changed = bChanged;
  return bChanged;
}

int gKtvTickCount = 0, gKtvPulseCount = 0;
void tsKtvBlk::SetTickValue() {
  GPIO_PinState cPinState = HAL_GPIO_ReadPin(paPinDesc[0].GPIO, (1 << paPinDesc[0].PinIdx));
  //Инвертируем, чтобы '1' означала включение выхода датчика КТВ
  uint8_t cPin = (cPinState == GPIO_PIN_SET) ? 0 : 1;
  
  int cPos, cIdx;
  uint64_t cBit = 1;
  switch (State) {
  case ksStart:    //Выдержка между опросами (только в тестовом режиме)
    if (--Counter <= 0) {
      State = ksStarted; //
    }
    break;
  case ksWait:     //Ожидание (только в рабочем режиме)
    break;
  case ksStPulse:  //Стартовый импульс
    if (--Counter <= 0) {
      Pin_Set(&paPinDesc[1], GPIO_PIN_SET);
#ifdef PRE_SYNC_INT //Учёт интервала перед поиском синхроимпульса
      Counter = TICK_NUM_TO_SYNC;
      State = ksSync;
#else
      BuffIdx = 0;
      State = ksRead;
#endif
      ++gKtvStartEndCount;
    }
    break;
  case ksSync:     //Синхронизация
    if (--Counter <= 0) {
      BuffIdx = 0;
      State = ksRead;
    }
    break;
  case ksRead:     //Чтение состояний датчиков RND
    cPos = BuffIdx / 64;
    cIdx = BuffIdx % 64;
    if (cPin) {
      Bitmap[cPos] |= (cBit << cIdx);
      if (cIdx > 30) {
        Bitmap[cPos] |= (cBit << cIdx);
      }
    } else {
      Bitmap[cPos] &= ~(cBit << cIdx);
    }
    ++BuffIdx;
    if (BuffIdx >= KTV_BITMAP_SIZE) { //KTV_BITMAP_SIZE_BY_NUM(gApp.pAppDesc->KtvSensNum)) {
      State = ksEnd;
    }
    break;
  }
  ++gKtvTickCount;
}

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"


extern "C" {
  extern uint64_t GetCurrMsecs();
}

//Проверка, есть ли сработавшие КТВ
bool tsKtvBlk::Triggered() {
  int16_t cKtvNum = gApp.pAppDesc->KtvSensNum;
  int16_t cDppNum = gApp.pAppDesc->DppSensNum;
  for (int i = 0; i < KTV_NUM_MAX + 1; ++i) {
    if (aKtvElem[i].EnumValue.Enum & KTV_TRIGGERED) {
      if ((i > 0) && (i <= cKtvNum))
        return true;
      if ((i >= KTV_DPP_POS) && (i < (KTV_DPP_POS + cDppNum)))
        return true;
    }
  }
  return false;
}

bool tsKtvBlk::ItemInWork(int iIdx) {
  int16_t cKtvNum = gApp.pAppDesc->KtvSensNum;
  int16_t cDppNum = gApp.pAppDesc->DppSensNum;
  if ((iIdx > 0) && (iIdx <= cKtvNum))
    return true;
  if ((iIdx >= KTV_DPP_POS) && (iIdx < (KTV_DPP_POS + cDppNum)))
    return true;
  return false;
}

void tsKtvBlk::ProcessKb() {
  static int8_t sbLastKbFault = 0;
#ifdef KTV_REPEAT_ON_KB_OFF
  if (!IsKbNorm()) {
    sbLastKbFault = 2;
    Counter = KTV_TEST_INTERVAL / KTV_TICK_IN_MSEC;
    State = ksStart;
  } else {
    if (sbLastKbFault > 1) {
      --sbLastKbFault;
      Counter = KTV_TEST_INTERVAL / KTV_TICK_IN_MSEC;
      State = ksStart;
    } else {
      if (sbLastKbFault > 0) {
        sbLastKbFault = 0;
        //Проверить, есть ли сработавшие КТВ
        if (Triggered()) {
          //Если есть сработавшие, то: ещё раз запустить проверку
          Counter = 500 / KTV_TICK_IN_MSEC;
          State = ksStart;
        } else {
          State = ksWait;
        }
      }
    }
  }
#else
 #ifdef KTV_TEST  //Включение КТВ в тестовом режиме (постоянный пуск процесса)
  Counter = KTV_TEST_INTERVAL / KTV_TICK_IN_MSEC;
  State = ksStart;
 #else
  if (!IsKbNorm()) {
    sbLastKbFault = true;
    State = ksWait;
  } else {
    if (sbLastKbFault) {
      sbLastKbFault = false;
      Counter = KTV_TEST_INTERVAL / KTV_TICK_IN_MSEC;
      State = ksStart;
    } else {
      State = ksWait;
    }
  }
 #endif
#endif
}

//Вызываемая из потока KTV функция
//Выполняется долго (5 - 7 сек для количества КТВ = 120; 1 - 2 сек для количества КТВ = 40)
uint64_t gKtvEndTime = 0, gKtvStartTime, gKtvCheckPeriod;
void tsKtvBlk::ProcessRead() {
//  Start();
  //Ожидание считывания состояний датчиков КТВ
  while (State != ksEnd) {
    vTaskDelay(10);
  }
  if (ProcessKtvProf()) {
    if (State == ksNoActive) { //Нет активных на связи: Очистить все КТВ
      int cNumber = KTV_NUM_MAX; //gApp.pAppDesc->KtvSensNum;
      for (int i = 0; i < (cNumber + 1); ++i) { //По всем КТВ:
        if (aKtvElem[i].EnumValue.Enum != 0) {
          aKtvElem[i].EnumValue.Enum = 0;
          aKtvElem[i].Changed = true;
          Changed = true;
        }
      }
      //Событие: Нет активных КТВ
    }
  }
#if (USE_USK_M == 0)
  extern void KtvToVoicing(uint64_t iKtvBreak);
  if (IsTriggered) {
    KtvToVoicing(Profile);
  }
#endif
  ProcessKb();
  gKtvCheckPeriod = GetTickCount() - gKtvStartTime;
  uint64_t cPeriod = GetTickCount() - StartTick;
  Period = cPeriod;
  CheckTime = GetCurrMsecs();
}

void tsKtvBlk::Process(eKtvMsg iMsg) {
  if (iMsg != kmNone) {
    if (iMsg == kmStart) {
      gKtvStates.AddState(State, iMsg);
      Start();
      ProcessRead();
    } else { //Конец интервала пуска или тестового интервала
      if (State == ksStarted) {
        gKtvStates.AddState(State, iMsg);
//        Start();
        ProcessRead();
      }
    }
  } else { //Конец интервала пуска или тестового интервала
    if (State == ksStarted) {
      gKtvStates.AddState(State, iMsg);
      Start();
      ProcessRead();
    }
  }
}

#include "cAppInfoTask.h"
bool tsKtvBlk::SetTo(tsKtvBlkInfo * ipKtvBlkInfo) {
  bool cChanged = false;
  for (int i = 0; i < (KTV_NUM_MAX + 1); ++i) {
    if (aKtvElem[i].EnumValue.Enum != ipKtvBlkInfo->aKtvElemInfo[i].Data) {
      aKtvElem[i].EnumValue.Enum = ipKtvBlkInfo->aKtvElemInfo[i].Data;
      cChanged = true;
    }
  }
  return cChanged;
}

//uint64_t tsKtvBlk::GetCurrKtvState() {
//  uint64_t cResult = 0, cBit = 0x01;
//  for (int i = 0; i < (KTV_NUM_MAX + 1); ++i) { //По всем КТВ:
//    if (aKtvElem[i].EnumValue.Enum & KTV_TRIGGERED)
//      cResult |= cBit;
//    cBit <<= 1;
//  }
//  Changed = false;
//  return cResult;
//}

#endif

// ===== Внешние вызовы из "C" =====
#ifdef __cplusplus
extern "C" {
#endif

//#define CHECK_KTV
void HAL_TIM_CaptureCallback(TIM_HandleTypeDef *htim)
{
#if (USE_KTV_BLOCK == 1)
  /* Prevent unused argument(s) compilation warning */
  if (htim == &htim14) {
    USK_WITH_KTV.KtvBlk.SetTickValue();
#ifdef CHECK_KTV
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
#endif
  }
#endif
}

bool KtvItemInWork(int iIdx) {
  return USK_WITH_KTV.KtvBlk.ItemInWork(iIdx);
}

#if (USE_KTV_BLOCK == 1)

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "queue.h"
QueueHandle_t gQueueKTV_InHandle;
eKtvMsg gKtvMsg;
void TaskKTV(void *argument) {
  //Стартовый пуск блока КТВ выполняется при инициализации УШК
  /* Infinite loop */
  for(;;) {
    BaseType_t cResult = xQueueReceive (gQueueKTV_InHandle, &gKtvMsg, (TickType_t)10);
    if (cResult != pdPASS)
      gKtvMsg = kmNone;
    if (gKtvEndTime == 0) {
      if ((gKtvMsg != kmNone) || (USK_WITH_KTV.KtvBlk.State == ksStarted)) {
        gKtvStartTime = GetTickCount();
        gKtvEndTime = KTV_MAX_POLL_INTERVAL;
        gKtvEndTime += gKtvStartTime;
      }
    } else {
      if (GetTickCount() >= gKtvEndTime) {
        gKtvEndTime = 0;
      }
      osDelay(10);
      continue;
    }
    USK_WITH_KTV.KtvBlk.Process(gKtvMsg);
    if (gKtvMsg != kmNone)
      gKtvMsg = kmFinish;
    osDelay(10);
  }
}

osThreadId_t TaskHandleKTV;
const osThreadAttr_t Task_attributes_KTV = {
  .name = "KtvTask",
  .stack_size = 128 * 20,
  .priority = (osPriority_t) osPriorityNormal1,
};

/* Definitions for gQueueKTV_In */
const osMessageQueueAttr_t gQueueKTV_In_attributes = {
  .name = "gQueueKTV_In"
};

void StartTaskKTV() {
  /* creation of gQueueKTV_In */
  gQueueKTV_InHandle = (QueueHandle_t)osMessageQueueNew (2, sizeof(uint16_t), &gQueueKTV_In_attributes);

  TaskHandleKTV = osThreadNew(TaskKTV, NULL, &Task_attributes_KTV);
}
#endif

#ifdef __cplusplus
}
#endif


