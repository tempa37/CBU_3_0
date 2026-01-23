/*
 * task_inp_def.h
 *
 *  Создан: 27 апр. 2021 г.
 *      Автор: Сосновских А.А.
 */

/**
 * Определения для входов/выходов прикладной задачи
**/

#ifndef TASK_INP_DEF_H_
#define TASK_INP_DEF_H_

#include "stdint.h"
#include "stdbool.h"
#include "io_pin_desc.h"
#include "task_blocks_use.h"
#include "cAppDefs.h"


//
#define DEF_FLASH_WORD 0xffffffff


#define INP_POLL_CYCLE  (2) //Цикл опроса входов в мс (или тиках ОС, если придётся)

#if (USE_KTV_BLOCK == 1)

  #define INP_NUM_KTV         (50)  //Максимальное количество КТВ

  // ======= Блок входов типа КТВ
  // Квант - условная единица времени, в которой отсчитываются интервалы
  // Тик - интервал опроса линии КТВ
  #define PIN_NUM_KTV         ( 2)  //Количество пинов в блоке КТВ
  #define TICK_NUM_PER_QUANT   (2)  //Количество тиков на квант
  #define QUANT_NUM_KTV_BIT    (4)  //Количество квантов на один бит (Online, Break) 
  #define TICK_NUM_KTV_BIT    (QUANT_NUM_KTV_BIT * TICK_NUM_PER_QUANT) //8 Количество тиков на один бит
  #define QUANT_NUM_KTV_PAUSE  (3)  //Количество квантов в паузе
  #define TICK_NUM_KTV_PAUSE  (QUANT_NUM_KTV_PAUSE * TICK_NUM_PER_QUANT)  // 4 Количество тиков в паузе
  #define QUANT_NUM_KTV       ((QUANT_NUM_KTV_BIT * 2) + QUANT_NUM_KTV_PAUSE) //11 Количество квантов на один КТВ
  #define TICK_NUM_KTV        (QUANT_NUM_KTV * TICK_NUM_PER_QUANT) //22 Количество тиков на один КТВ
  #define QUANT_NUM_KTV_START (16) //16 Количество квантов в стартовом блоке
  #define TICK_NUM_KTV_START  (QUANT_NUM_KTV_START * TICK_NUM_PER_QUANT)  //64 Количество тиков в стартовом блоке
                                                                          // = момент включения КТВ = начало отсчёта
  #define TICK_NUM_KTV_ON     (TICK_NUM_KTV_START - TICK_NUM_KTV_PAUSE) //140 Количество тиков в импульсе включения КТВ
                                                                        // = момент выключения КТВ = начало синхроимпульса
#endif

// ======= Блок входов типа ОС
#define INP_NUM_BK  (2)
#define INP_SIZE_BK (4)
//Индексы пинов входов БК
#define BK_ANA_RELE_IDX (0)
#define BK_MCU_BLK_1_IDX (1)
#define BK_MCU_BLK_2_IDX (2)
#define BK_BDU_M_S_IDX (3)

//Количество пинов общих для блока БК
#define INP_SIZE_BK_COMMON (3)
//Индексы общих пинов БК
#define BK_DIF_SIG_OUT_IDX (INP_NUM_BK * INP_SIZE_BK)
#define BK_STM_LOOP_LINK_IDX (BK_DIF_SIG_OUT_IDX + 1)
#define BK_STM_CUTOFF_IDX (BK_DIF_SIG_OUT_IDX + 2)


// ======= Блок входов дискретных
#define INP_NUM_DI (8) //Количество входов типа ОС

// ======= Блок входов типа Значение -> ОС (Namur)
#define INP_NUM_NAM  (2)

// ======= Блок входов типа Значение
#define INP_NUM_ANA  (6)

// ======= Блок входов типа Частота
#define INP_NUM_FREQ (1)

// ======= Блок управляющих кнопок
#define INP_NUM_CTRL  (16)

// ======= Блок выходов типа реле
#define OUT_NUM_RELE  (5)
#define OUT_SIZE_RELE (2)

// ======= Блок выходов светодиодных
#define OUT_NUM_LED   (9)


enum eUnitPos {  //Позиция УШК (или иного устройства) в системе управления (конвейером)
  upDist,     //Распределён по механизму
  upMain,     //Главный привод. Не может быть выключен из работы
  upUskM1,    //УШК-мини №1  //Головной барабан. Может быть выключен из работы
  upUskM2,    //УШК-мини №2  //Промпривод. Может быть выключен из работы
  upUskM3,    //УШК-мини №3  //Промпривод 2. Может быть выключен из работы
  upUskM4,    //УШК-мини №4  //Хвостовой. Может быть выключен из работы
  upCount
};

enum eElemClass { //Класс элемента (по типу выхода / входа)
  eecNone, //Неизвестный
  eecElem, //Элемент простой
  eecEnum, //Перечисление
  eecAna,  //Аналоговый
  eecRele, //Реле
  eecCount
};

enum eEnvElemType { //Тип элемента
  etNone,       //Тип элемента не определён (или для блока с разными типами элементов)
  etSensBK,     //Вход типа Контур безопасности                 Enum
  etSensDI,     //Вход типа ОС или дискретный вход              Enum
  etSensNAM,    //Namur: Вход аналоговый, результат - частотный Ana
  etSensANA,    //Аналоговый: Токовый, Напряжения, PT100, NAMUR Ana
  etSensFREQ,   //Вход типа Частота                             Enum

  etButtCTRL,   //Элемент управления (кнопка)                   Elem

  etOutRELE,    //Выход реле                                    Elem
  etOutLED,     //Выход светодиод                               Elem

  etCountIO,
  
  etUsk,        //УШК
  etUskM,       //УШК мини
  etCountAll
};


enum eIO_Type { //Тип входа/выхода
  iotBK,    //Вход типа блок-контакт
  iotDI,    //Вход типа ОС (= сухой контакт, или дискретный вход)
  iotANA,   //Аналоговый: Токовый, Напряжения, PT100, NAMUR
  iotCTRL,  //Вход управления (кнопки)
  iotRELE,  //Выход типа реле (+ диод)
  iotLED,   //Выход типа светодиод
  iotCount
};

enum eElemState {
  asErrorLow,     //  0 Выход за нижний предел возможных значений датчика (= неисправность аппаратуры)
  asFaultLow,     //  1 Уровень срабатывания аналогового датчика нижний
  asWarningLow,   //  2 Уровень предупреждения аналогового датчика нижний
  asNormal,       //  3 Нормальное состояние датчика
  asWarningHigh,  //  4 Уровень предупреждения аналогового датчика верхний
  asFaultHigh,    //  5 Уровень срабатывания аналогового датчика верхний
  asErrorHigh,    //  6 Выход за верхний предел возможных значений датчика (= неисправность аппаратуры)
  asCountAna,     //  7 Количество состояний аналоговых датчиков
  //Датчики типа ОС
  asTrigger,      //  8 Состояние "Сработал" (например, обрыв датчика ОС)
  asFault,        //  9 Состояние "Авария" = невозможность нормальной работы (КЗ для датчика ОС)
  asHardError,    // 10 Ошибка (= неисправность аппаратуры) = Невозможные состояния датчика
  asIsOff,        // 11 Ошибка = контур отключен
  asIsDiff,       // 12 Ошибка = разность уставок ПО и тумблера SB2.1
  //Ошибки реле
  asFaultNoOn,    // 13 Состояние = НЕВКЛючение реле
  asFaultNoOff,   // 14 Состояние = НЕВЫКЛючение реле
  asCountAll
};

enum eValueType { //Тип значения    (Размер)
  vtBool,   //Логический                1
  vtEnum,   //Перечисление              1
  vtBitmap, //Битовая карта             8
  vtInt8,   //Целое со знаком байт      1
  vtUint8,  //Целое без знака байт      1
  vtInt16,  //Целое со знаком слово     2
  vtUint16, //Целое без знака слово     2
  vtInt32,  //Целое со знаком 4х слово  4
  vtUint32, //Целое без знака 4х слово  4
  vtFloat,  //Плавающая запятая         4
  vtCount
};

#define V_U_MAX f_u32

#pragma pack(1)
typedef struct sEnumValue {
  uint8_t     Enum;   //Состояние значения
  uint16_t    Count;  //Счётчик текущего состояния
} tsEnumValue;

//Структура для обработки логических значений пинов
typedef struct sPinValue {
  uint8_t   bValue:1; //Текущее логическое значение (0 или 1)
  uint8_t   Error :1; //Ошибка (невозможное состояние)
  uint8_t   Count :5; //Счётчик состояния
  uint8_t   Pulse :1; //Вход: Наличие последовательности / Выход: Подключение диода
} tsPinValue;

typedef struct sVal {
  union {
    uint8_t     f_Data[4];
    uint8_t     f_Enum;
    int16_t     f_i16;
//    uint16_t    f_u16;
    int32_t     f_i32;
    uint32_t    f_u32;
//    float       f_float;
    tsEnumValue f_EnumValue;
    tsPinValue  f_PinValue[4];
  };
} tsVal;


typedef struct sValue {
  union {
    uint8_t     f_Data[4];
    uint8_t     f_Enum;
    int16_t     f_i16;
//    uint16_t    f_u16;
    int32_t     f_i32;
    uint32_t    f_u32;
//    float       f_float;
    tsEnumValue f_EnumValue;
    tsPinValue  f_PinValue[4];
  };
  uint8_t eState;
} tsValue;
#pragma pack()

#include "um_defs.h"
//Блок информации о модулях УРМ, и реле модулей УРМ, входящих в контур безопасности
typedef struct sReleSafeContUsk {
  //[0] = собственные реле (только в УШК)
  //[1..6] = модули УРМ, и реле в модулях УРМ
  uint8_t UrmBitmap;  //Битовая карта УРМ, вставленных в УШК / УШК-М (
  uint8_t aSafe[UMODULE_NUM + 1]; //Блок витовых карт реле модулей УРМ, входящих в контур безопасности
} tsReleSafeContUsk;

typedef struct sReleSafeContApp {
  tsReleSafeContUsk aReleSafeContUsk[upCount - upMain]; // Для УШК (= 0) + 4 * УШК-М (= 1..4)
} tsReleSafeContApp;

//Состояние блока БК (контура безопасности) (2 входа)
typedef struct sBkBlkState {
  union {
    struct {
      uint8_t Bdu1_M_S    : 1;  //Вкл канала 1 контура безопасности
      uint8_t Bdu2_M_S    : 1;  //Вкл канала 2 контура безопасности
      uint8_t LoopTumbler : 1;  //Состояние тумблера Loop
      uint8_t LoopLink    : 1;  //Выход уставки ПО Loop
      uint8_t SettingDiff : 1;  //Вход разности уставок
      uint8_t CutOff      : 1;  //Выход отключения (разрыва) контуров
    };
    uint8_t Data;
  };
} tsBkBlkState;


//Состояния СКАН-интеркомов
#pragma pack(1)

typedef struct sIComState { //Состояние интеркома
  union {
    struct {
      uint16_t  VoiceActing   :1; //0 – озвучка голоса
      uint16_t  PpsActing     :1; //1 – озвучка ППС
      uint16_t  MicroOn       :1; //2 – микрофон включен
      uint16_t  CallIsOn      :1; //3 – вызов включен
      uint16_t  IComInline    :1; //4 – активная работы Интеркома в линию
      uint16_t  IComConnected :1; //5 – интерком на связи
      uint16_t  Reserved      :1; //6 – резерв
      uint16_t  IsCharging    :1; //7 – заряд аккум. Включен
        //Ошибки говорушки (при бит 6=1) :
      uint16_t  NoCharging    :1; //8 – аккумулятора не заряжается = большое сопр. (издох),
      uint16_t  NoBattery     :1; //9 – нет аккумулятора <2Вольт,
      uint16_t  NoVoiceLine   :1; //10 – нет линии голоса,
      uint16_t  No12V_Line    :1; //11 – нет линии 12В,
      uint16_t  NoSCAN_Line   :1; //12 – нет линии СКАН,
      uint16_t  KzSCAN_Line   :1; //13 – КЗ линии СКАН,
      uint16_t  Kz12V_Line    :1; //14 – КЗ линии 12В,
      uint16_t  KtvNoWithICom :1; //15 – КТВ без интеркома.
        //Значения
      uint8_t IComVoltage;        //Напряжение в 0.1В
      uint8_t IComCurrent;        //Ток в 0.01А
    };
    uint32_t  Data;
  };
} tsIComState;

#pragma pack()

#include "string.h"
typedef struct sIComStateBlock { //Состояния интеркомов
  tsIComState  aIComState[SCAN_ICOM_REG_NUM];
  uint8_t  aIComStateConnect[SCAN_ICOM_NUM];
  void Init() {
    memset(this, 0, sizeof(sIComStateBlock));
  }
} tsIComStateBlock;



#endif // TASK_INP_DEF_H_
