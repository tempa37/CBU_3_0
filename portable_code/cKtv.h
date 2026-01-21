/*
 * cKtv.h
 *
 *  Created on: 26 мая 2022 г.
 *      Author: Сосновских А.А.
 *
 *      Элементы окружения задачи: входы, управление, выходы
 */

#ifndef KTV_H_
#define KTV_H_

#include "task_inp_def.h"

//Последовательность адресации (в мс):
// Стартовый  64 
// Пауза     150
// Синхро      8
// Пауза      32
// Адресный интервал:
// Пауза м/у КТВ  32
// На связи       16
// Сработал       16


#if (USE_KTV_BLOCK == 1)
// ===== Определения КТВ =====
#define KTV_REPEAT_ON_KB_OFF  //Включение КТВ периодически, пока контур разорван (приоритет перед KTV_TEST)

//#define KTV_TEST  //Включение КТВ в тестовом режиме (постоянный пуск процесса)
#define PRE_SYNC_INT //Учёт интервала перед поиском синхроимпульса

enum eKtvMsg { //Сообщение блока КТВ
  kmNone,     //Нет
  kmStart,    //Пуск процесса обнаружения (сработавших, или отключенных) КТВ
  kmFinish,   //Конец
  kmCount
};

enum eKtvFlag { //Флаги состояния датчика КТВ
  kfOnline,     //На связи
  kfTriggered,  //Сработал
  kfError,      //Ошибка
  kfCount
};

#define KTV_ONLINE    (0x01 << kfOnline)
#define KTV_TRIGGERED (0x01 << kfTriggered)
#define KTV_ERROR     (0x01 << kfError)

//Включение стартового импульса
#define KTV_START_PULSE

//Размер битовой карты для записи профиля состояний датчиков КТВ
#define KTV_BITMAP_SIZE (((KTV_NUM_MAX + 1) * TICK_NUM_KTV) + TICK_NUM_KTV_START + 128)

#define KTV_BITMAP_SIZE_BY_NUM(iNum) (((iNum + 1) * TICK_NUM_KTV) + TICK_NUM_KTV_START + 128)

//Размер тика КТВ в мсек
#define KTV_TICK_IN_MSEC  (2)
//Пауза перед считыванием профилей датчиков КТВ в тиках
#define KTV_PAUSE_IN_TICKS (16)

//Экспериментально найденная пауза после старотвого импульса
#define KTV_START_PAUSE_IN_TICKS (20)

//Интервал начала работы КТВ после включения питания
#define KTV_BOOTUP_INTERVAL  (5000)
//Интервал между пусками КТВ в тестовом режиме
#define KTV_TEST_INTERVAL  (5000)

#define SYNC_PULSE_OFFSET_IN_TICK  (75)
#define SYNC_PULSE_WIDTH_IN_TICK  4

#define KTV_MAX_POLL_INTERVAL ((KTV_NUM_MAX + 2) * TICK_NUM_KTV * KTV_TICK_IN_MSEC) + \
            (KTV_PAUSE_IN_TICKS + TICK_NUM_KTV_START + SYNC_PULSE_OFFSET_IN_TICK + SYNC_PULSE_WIDTH_IN_TICK) * KTV_TICK_IN_MSEC;

//Собственный блок КТВ
#if (USE_USK_M == 0) //На УШКМ
  #define USK_WITH_KTV  gKts.aUsk[0]
#else
  #define USK_WITH_KTV  gKtsM.UskM
#endif

#if (USE_USK_M == 0) //На УШКМ
  #define USK_M_WITH_KTV(PDrvIdx)  gKts.aUskM[PDrvIdx]
#else
  #define USK_M_WITH_KTV(PDrvIdx)  gKtsM.UskM
#endif

//Тип физического датчика КТВ
enum eKtvSensType {
  kstNone,     //Неопределен тип
  kstActive,   //Активный датчик
  kstPassive,  //Пассивный датчик
  kstCount
};

typedef struct sKtvElemDesc { //Дескриптор элемента КТВ
  uint8_t       Idx    : 7; //Индекс элемента в пределах собственного типа
  uint8_t       Present: 1; //Статус элемента: Имеется фактически (для фиксированных* = 1)
  enum eKtvSensType  SensType;   //Тип физического датчика КТВ
} tsKtvElemDesc;

typedef struct sKtvBlkDesc {
  uint16_t      SelfSize;     //Собственный размер
  uint8_t       Status;       //Статус (set of bit):
  enum eUnitPos UnitPos;      //Размещение блока
  int16_t       Tick;         //Размер цикла опроса в мс. (0 = нет цикла, вызов по запросу)
  // !!! Number равно pAppDesc->KtvSensNum
//  uint8_t       Number;       //Количество датчиков КТВ в блоке (настраивается, в пределех до 50)
//  uint8_t       FullNumber;   //Количество входов всего в блоке (50)
  tsKtvElemDesc aKtvElemDesc[KTV_NUM_MAX + 1];  //Массив дескрипторов элементов
} tsKtvBlkDesc;


enum eKtvState { //Состояние блока КТВ
  ksNone,     //Начальное
  ksStart,    //Предпусковой интервал (только в тестовом режиме)
  ksStarted,  //Предпусковой интервал завершен
  ksWait,     //Ожидание (только в рабочем режиме)
  ksStPulse,  //Стартовый импульс
  ksSync,     //Выдержка перед началом поиска синхроимпульса
  ksRead,     //Чтение состояний датчиков RND
  ksEnd,      //Конец чтения
  ksNoActive, //Нет активных КТВ
  ksCount,
};

typedef struct sKtvElem { //Элемент КТВ
  //const tsKtvElemDesc * KtvElemDesc;  //Дескриптор
  tsEnumValue EnumValue;              //Значение
  bool    Changed;                    //Изменен
  void Init() {
    EnumValue.Enum = ksNone;
//    EnumValue.Count = 0;
  }
} tsKtvElem;

typedef struct sKtvBlkInfo tsKtvBlkInfo;
typedef struct sKtvBlk { //Блок КТВ
  eKtvState       State;
  int32_t         Counter;  //Счётчик интервалов
  int64_t         BuffIdx;  //Индекс в буфере состояния
  uint64_t        Bitmap[KTV_BITMAP_SIZE / 64 + 1];
//#ifdef STM32H7xx_HAL_GPIO_H
  const tsPinDesc * paPinDesc;
//#endif
  tsKtvBlkDesc  * pKtvBlkDesc;
  tsKtvElem       aKtvElem[KTV_NUM_MAX + 1];
  bool            Changed;
  uint64_t        StartTick;
  int32_t         Period;
  uint64_t        ReadEndTime;  //Момент конца опроса для датчиков КТВ
  uint64_t        CheckTime;    //Момент завершения процесса считывания
  uint64_t        Profile;      //Профиль срабатывания КТВ, перепроезда
  bool            IsTriggered;  //Признак срабатывания КТВ, перепроезда
  
  void Init(tsKtvBlkDesc * ipKtvBlkDesc);
  bool IsFinished() {
    if ((State == ksWait) || (State >= ksEnd))
      return true;
    return false;
  }
  void Start();
  uint64_t GetCurrKtvProf(int iBitmapIdx);
  void ClearKtvResult();
  bool ProcessKtvProf();
  void SetTickValue();
  bool Triggered();
  bool ItemInWork(int iIdx);
  void ProcessKb();
  void ProcessRead();
  void Process(eKtvMsg iMsg);
//  uint64_t GetCurrKtvState();
  
  bool SetTo(tsKtvBlkInfo * ipKtvBlkInfo);
} tsKtvBlk;


// + === Для отладки адресного модуля КТВ ===
typedef struct sKtvRezult { //Блок результатов КТВ
  uint8_t   Before; //Количество тиков перед Online
  uint8_t   Online; //Количество тиков в Online
  uint8_t   Break;  //Количество тиков в Break
  uint8_t   After;  //Количество тиков после Break
} tsKtvRezult;
// - === Для отладки адресного модуля КТВ ===
#endif

#endif //KTV_H_
