/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ktv.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define I2C_EXPANDER16_ADDR 0x21U /* Адрес 16-портового расширителя (A0=1, A1=0, A2=0) */
//#define I2C_EXPANDER8_ADDR  0x20U /* Адрес 8-портового расширителя (A0=0, A1=0, A2=0) */

#define I2C_EXPANDER16_ADDR (0x21U << 1) /* Адрес 16-портового расширителя (A0=1, A1=0, A2=0) */
#define I2C_EXPANDER8_ADDR  (0x20U << 1)  /* Адрес 8-портового расширителя (A0=0, A1=0, A2=0) */

#define I2C_EXPANDER16_RX_SIZE 2U /* Два байта: p0..p7 и p8..p15 */
#define I2C_EXPANDER8_RX_SIZE  1U /* Один байт: p0..p7 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} ExpanderPinMap;

static const ExpanderPinMap expander16_map[16] =
{
  {OE_RELE_GPIO_Port, OE_RELE_Pin},             /* p0 = OE_RELE */
  {Rele_5_GPIO_Port, Rele_5_Pin},               /* p1 = RELE_5 */
  {PWR_KTV_BUF_GPIO_Port, PWR_KTV_BUF_Pin},     /* p2 = PWR_KTV */
  {NULL, 0U},                                   /* p3 = не используется */
  {NULL, 0U},                                   /* p4 = не используется */
  {Rele_4_GPIO_Port, Rele_4_Pin},               /* p5 = RELE_4 */
  {NULL, 0U},                                   /* p6 = не используется */
  {Rele_3_GPIO_Port, Rele_3_Pin},               /* p7 = RELE_3 */
  {NULL, 0U},                                   /* p8 = не используется */
  {SD_SW_GPIO_Port, SD_SW_Pin},                 /* p9 = SD_SW */
  {NULL, 0U},                                   /* p10 = не используется */
  {Rele_1_GPIO_Port, Rele_1_Pin},               /* p11 = RELE_1 */
  {NULL, 0U},                                   /* p12 = не используется */
  {Rele_2_GPIO_Port, Rele_2_Pin},               /* p13 = RELE_2 */
  {DISP_LIGHT_BUF_GPIO_Port, DISP_LIGHT_BUF_Pin}, /* p14 = DISP_LIGHT_EN */
  {NULL, 0U}                                    /* p15 = не используется */
};

static const ExpanderPinMap expander8_map[8] =
{
  {NULL, 0U},                                   /* p0 = не используется */
  {NULL, 0U},                                   /* p1 = не используется */
  {NULL, 0U},                                   /* p2 = не используется */
  {NULL, 0U},                                   /* p3 = не используется */
  {MUX_SEL_GPIO_Port, MUX_SEL_Pin},             /* p4 = MUX_SEL */
  {NULL, 0U},                                   /* p5 = не используется */
  {NULL, 0U},                                   /* p6 = не используется */
  {NULL, 0U}                                    /* p7 = не используется */
};

static uint8_t i2c_rx_buffer[I2C_EXPANDER16_RX_SIZE];
static uint8_t i2c_tx_buffer[I2C_EXPANDER16_RX_SIZE];
static uint16_t expander16_state = 0U;
static uint8_t expander8_state = 0U;
static uint8_t active_i2c_address = 0U;
static const uint16_t expander16_input_mask = (1U << 9); /* p9 = SD_SW (input) */
static GPIO_PinState sd_sw_last_state = GPIO_PIN_RESET;
static GPIO_PinState sd_sw_return_state = GPIO_PIN_RESET;
static uint8_t pcf_int_latched = 0U;
volatile GPIO_PinState door = GPIO_PIN_RESET;
volatile GPIO_PinState error_sv = GPIO_PIN_RESET;
volatile GPIO_PinState on_bkk_k1 = GPIO_PIN_RESET;
volatile GPIO_PinState on_bkk_k2 = GPIO_PIN_RESET;
volatile uint8_t direction1 = 0U;
volatile uint8_t direction2 = 0U;
volatile uint8_t bkk_k1_poll_required = 0U;
volatile uint8_t bkk_k2_poll_required = 0U;



static inline void EXTI_DisableLine(uint16_t pinmask);
static inline void EXTI_EnableLine(uint16_t pinmask);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t active_rx_size = 0U;
static volatile uint32_t tim2_tick_count = 0U;
static uint8_t on_3v3_enabled = 0U;

#define ON_3V3_ENABLE_DELAY_MS (10000U)
#define ON_3V3_ENABLE_DELAY_TICKS (ON_3V3_ENABLE_DELAY_MS / KTV_TICK_IN_MSEC)

/* Буфер/анализ оптронов: выбор 250 мс окна при частоте 1 кГц. */
#define OPTRON_SAMPLE_RATE_HZ 1000U
#define OPTRON_FRAME_MS 250U
#define OPTRON_EVAL_PERIOD_MS 1000U
#define OPTRON_FRAME_SAMPLES ((OPTRON_SAMPLE_RATE_HZ * OPTRON_FRAME_MS) / 1000U)
#define OPTRON_EVAL_TICKS ((OPTRON_SAMPLE_RATE_HZ * OPTRON_EVAL_PERIOD_MS) / 1000U)
#define OPTRON_NOISE_MIN_PERCENT 40U
#define OPTRON_NOISE_MAX_PERCENT 60U

typedef enum
{
  OPTRON_STATE_LOW = 0U,
  OPTRON_STATE_HIGH = 1U,
  OPTRON_STATE_NOISE = 2U
} OptronState;

/* Биты в выборке для четырёх оптронов. */
#define OPTRON_SAMPLE_1_1 (1U << 0)
#define OPTRON_SAMPLE_1_2 (1U << 1)
#define OPTRON_SAMPLE_2_1 (1U << 2)
#define OPTRON_SAMPLE_2_2 (1U << 3)

static volatile uint8_t optron_samples[OPTRON_FRAME_SAMPLES];
static volatile uint16_t optron_sample_index = 0U;
static volatile uint16_t optron_cycle_tick = 0U;
static volatile uint8_t optron_eval_pending = 0U;

/* Текущее состояние входов оптронов после фильтрации. */
volatile OptronState optron1_1_state = OPTRON_STATE_LOW;
volatile OptronState optron1_2_state = OPTRON_STATE_LOW;
volatile OptronState optron2_1_state = OPTRON_STATE_LOW;
volatile OptronState optron2_2_state = OPTRON_STATE_LOW;

volatile uint8_t kontur1_kz = 0U;
volatile uint8_t kontur1_diod = 0U;
volatile uint8_t kontur1_obryv = 0U;
volatile uint8_t kontur2_kz = 0U;
volatile uint8_t kontur2_diod = 0U;
volatile uint8_t kontur2_obryv = 0U;

static void Optron_ProcessFrame(void)
{
  uint8_t frame_copy[OPTRON_FRAME_SAMPLES];
  uint16_t ones_1_1 = 0U;
  uint16_t ones_1_2 = 0U;
  uint16_t ones_2_1 = 0U;
  uint16_t ones_2_2 = 0U;

  if (optron_eval_pending == 0U)
  {
    return;
  }

  /* Копируем буфер быстро, чтобы не мешать прерыванию TIM1. */
  __disable_irq();
  for (uint16_t i = 0U; i < OPTRON_FRAME_SAMPLES; i++)
  {
    frame_copy[i] = optron_samples[i];
  }
  optron_eval_pending = 0U;
  __enable_irq();

  for (uint16_t i = 0U; i < OPTRON_FRAME_SAMPLES; i++)
  {
    uint8_t sample = frame_copy[i];
    if ((sample & OPTRON_SAMPLE_1_1) != 0U)
    {
      ones_1_1++;
    }
    if ((sample & OPTRON_SAMPLE_1_2) != 0U)
    {
      ones_1_2++;
    }
    if ((sample & OPTRON_SAMPLE_2_1) != 0U)
    {
      ones_2_1++;
    }
    if ((sample & OPTRON_SAMPLE_2_2) != 0U)
    {
      ones_2_2++;
    }
  }

  /* Классификация по заполнению: 40-60% = зашумленный меандр. */
  uint8_t percent_1_1 = (uint8_t)((ones_1_1 * 100U) / OPTRON_FRAME_SAMPLES);
  uint8_t percent_1_2 = (uint8_t)((ones_1_2 * 100U) / OPTRON_FRAME_SAMPLES);
  uint8_t percent_2_1 = (uint8_t)((ones_2_1 * 100U) / OPTRON_FRAME_SAMPLES);
  uint8_t percent_2_2 = (uint8_t)((ones_2_2 * 100U) / OPTRON_FRAME_SAMPLES);

  optron1_1_state = (percent_1_1 < OPTRON_NOISE_MIN_PERCENT) ? OPTRON_STATE_LOW :
                    (percent_1_1 > OPTRON_NOISE_MAX_PERCENT) ? OPTRON_STATE_HIGH :
                    OPTRON_STATE_NOISE;
  optron1_2_state = (percent_1_2 < OPTRON_NOISE_MIN_PERCENT) ? OPTRON_STATE_LOW :
                    (percent_1_2 > OPTRON_NOISE_MAX_PERCENT) ? OPTRON_STATE_HIGH :
                    OPTRON_STATE_NOISE;
  optron2_1_state = (percent_2_1 < OPTRON_NOISE_MIN_PERCENT) ? OPTRON_STATE_LOW :
                    (percent_2_1 > OPTRON_NOISE_MAX_PERCENT) ? OPTRON_STATE_HIGH :
                    OPTRON_STATE_NOISE;
  optron2_2_state = (percent_2_2 < OPTRON_NOISE_MIN_PERCENT) ? OPTRON_STATE_LOW :
                    (percent_2_2 > OPTRON_NOISE_MAX_PERCENT) ? OPTRON_STATE_HIGH :
                    OPTRON_STATE_NOISE;

  /* Таблица соответствий для контура 1. */
  uint8_t optron1_1_meander = (optron1_1_state == OPTRON_STATE_NOISE) ? 1U : 0U;
  uint8_t optron1_2_meander = (optron1_2_state == OPTRON_STATE_NOISE) ? 1U : 0U;
  uint8_t optron1_1_high = (optron1_1_state == OPTRON_STATE_HIGH) ? 1U : 0U;
  uint8_t optron1_2_high = (optron1_2_state == OPTRON_STATE_HIGH) ? 1U : 0U;
  uint8_t bkk_k1_high = (on_bkk_k1 == GPIO_PIN_SET) ? 1U : 0U;

  kontur1_kz = 0U;
  kontur1_diod = 0U;
  kontur1_obryv = 0U;

  if (bkk_k1_high == 1U)
  {
    if ((optron1_1_meander == 1U) && (optron1_2_meander == 1U))
    {
      kontur1_kz = 1U;
    }
    else if ((optron1_1_high == 1U) && (optron1_2_high == 1U))
    {
      kontur1_obryv = 1U;
    }
  }
  else
  {
    if (((optron1_1_meander == 1U) && (optron1_2_high == 1U)) ||
        ((optron1_1_high == 1U) && (optron1_2_meander == 1U)))
    {
      kontur1_diod = 1U;
    }
  }
  
  
  /* Таблица соответствий для контура 2. */
  uint8_t optron2_1_meander = (optron2_1_state == OPTRON_STATE_NOISE) ? 1U : 0U;
  uint8_t optron2_2_meander = (optron2_2_state == OPTRON_STATE_NOISE) ? 1U : 0U;
  uint8_t optron2_1_high = (optron2_1_state == OPTRON_STATE_HIGH) ? 1U : 0U;
  uint8_t optron2_2_high = (optron2_2_state == OPTRON_STATE_HIGH) ? 1U : 0U;
  uint8_t bkk_k2_high = (on_bkk_k2 == GPIO_PIN_SET) ? 1U : 0U;

  kontur2_kz = 0U;
  kontur2_diod = 0U;
  kontur2_obryv = 0U;

  if (bkk_k2_high == 1U)
  {
    if ((optron2_1_meander == 1U) && (optron2_2_meander == 1U))
    {
      kontur2_kz = 1U;
    }
    else if ((optron2_1_high == 1U) && (optron2_2_high == 1U))
    {
      kontur2_obryv = 1U;
    }
  }
  else
  {
    if (((optron2_1_meander == 1U) && (optron2_2_high == 1U)) ||
        ((optron2_1_high == 1U) && (optron2_2_meander == 1U)))
    {
      kontur2_diod = 1U;
    }
  }
  
  
  
}

static void Expander_WritePin(const ExpanderPinMap *map, uint8_t index, GPIO_PinState state)
{
  if (map[index].port != NULL)
  {
    HAL_GPIO_WritePin(map[index].port, map[index].pin, state);
  }
}

static void ApplyExpander16State(uint16_t value)
{
  uint8_t oe_enabled = (value & 0x0001U) ? 1U : 0U;
  uint16_t masked_value = value;

  /* Остальные пины могут быть в 1 только при OE_RELE=0 */
  if (oe_enabled == 1U)
  {
    masked_value &= 0x0001U;
  }
  masked_value &= (uint16_t)(~expander16_input_mask);

  for (uint8_t i = 0U; i < 16U; i++)
  {
    if ((expander16_input_mask & (1U << i)) == 0U)
    {
      GPIO_PinState state = (masked_value & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
      Expander_WritePin(expander16_map, i, state);
    }
  }

  expander16_state = masked_value;
}

static void ApplyExpander8State(uint8_t value)
{
  for (uint8_t i = 0U; i < 8U; i++)
  {
    GPIO_PinState state = (value & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    Expander_WritePin(expander8_map, i, state);
  }

  expander8_state = value;
}

static void PrepareExpanderTx(uint8_t address)
{
  if (address == I2C_EXPANDER16_ADDR)
  {
    uint16_t input_value = 0U;

    if ((expander16_input_mask & (1U << 9)) != 0U)
    {
      input_value |= (HAL_GPIO_ReadPin(SD_SW_GPIO_Port, SD_SW_Pin) == GPIO_PIN_SET) ? (1U << 9) : 0U;
    }

    expander16_state = (expander16_state & ~expander16_input_mask) | (input_value & expander16_input_mask);
    //expander16_state = 0xF0F0;
    i2c_tx_buffer[0] = (uint8_t)(expander16_state & 0x00FFU);
    i2c_tx_buffer[1] = (uint8_t)((expander16_state >> 8) & 0x00FFU);
  }
  else if (address == I2C_EXPANDER8_ADDR)
  {
    i2c_tx_buffer[0] = expander8_state;
  }
}

static void SetPcfIntLevel(GPIO_PinState level)
{
  /* PCF_INT работает как open-drain: лог.1 = отпущен, лог.0 = тянем к земле. */
  HAL_GPIO_WritePin(PCF_INT_GPIO_Port, PCF_INT_Pin, level);
}

static void ClearPcfIntLatch(void)
{
  if (pcf_int_latched != 0U)
  {
    pcf_int_latched = 0U;
    SetPcfIntLevel(GPIO_PIN_SET);
  }
}

void UpdateBkkDirections(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (direction1 == 1U)
  {
    GPIO_InitStruct.Pin = On_BKK_k1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    EXTI_DisableLine(On_BKK_k1_Pin);
    HAL_GPIO_Init(On_BKK_k1_GPIO_Port, &GPIO_InitStruct);
    bkk_k1_poll_required = 0U;

  }
  else
  {
    GPIO_InitStruct.Pin = On_BKK_k1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(On_BKK_k1_GPIO_Port, &GPIO_InitStruct);
    bkk_k1_poll_required = 1U;
    EXTI_EnableLine(On_BKK_k1_Pin);
  }

  if (direction2 == 1U)
  {
    GPIO_InitStruct.Pin = On_BKK_k2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    EXTI_DisableLine(On_BKK_k2_Pin); 
    HAL_GPIO_Init(On_BKK_k2_GPIO_Port, &GPIO_InitStruct);
    bkk_k2_poll_required = 0U;
    
  }
  else
  {
    GPIO_InitStruct.Pin = On_BKK_k2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(On_BKK_k2_GPIO_Port, &GPIO_InitStruct);
    bkk_k2_poll_required = 1U;
    EXTI_EnableLine(On_BKK_k2_Pin);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_EnableListen_IT(&hi2c1);
  ApplyExpander16State(0U);
  ApplyExpander8State(0U);
  sd_sw_last_state = HAL_GPIO_ReadPin(SD_SW_GPIO_Port, SD_SW_Pin);
  sd_sw_return_state = sd_sw_last_state;
  pcf_int_latched = 0U;
  SetPcfIntLevel(GPIO_PIN_SET);
  Ktv_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Обработка результатов измерения оптронов в основном цикле. */
    Optron_ProcessFrame();
    Ktv_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = I2C_EXPANDER16_ADDR;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = I2C_EXPANDER8_ADDR;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUX_SEL_GPIO_Port, MUX_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Sv_kont_p_Pin|OE_RELE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Break_K_p_Pin|PCF_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Rele_1_Pin|Rele_5_Pin|DISP_LIGHT_BUF_Pin|PWR_KTV_BUF_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ON_3_3V_GPIO_Port, ON_3_3V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Rele_2_Pin|Rele_3_Pin|Rele_4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MUX_SEL_Pin */
  GPIO_InitStruct.Pin = MUX_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUX_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DOOR_Pin */
  GPIO_InitStruct.Pin = DOOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Optron1_2_Pin Optron1_1_Pin */
  GPIO_InitStruct.Pin = Optron1_2_Pin|Optron1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Sv_kont_p_Pin */
  GPIO_InitStruct.Pin = Sv_kont_p_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Sv_kont_p_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : On_BKK_k1_Pin On_BKK_k2_Pin error_sv_Pin SD_SW_Pin */
  GPIO_InitStruct.Pin = On_BKK_k1_Pin|On_BKK_k2_Pin|error_sv_Pin|SD_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Break_K_p_Pin */
  GPIO_InitStruct.Pin = Break_K_p_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Break_K_p_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Optron2_2_Pin Optron2_1_Pin */
  GPIO_InitStruct.Pin = Optron2_2_Pin|Optron2_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Rele_1_Pin Rele_5_Pin DISP_LIGHT_BUF_Pin ON_3_3V_Pin */
  GPIO_InitStruct.Pin = Rele_1_Pin|Rele_5_Pin|DISP_LIGHT_BUF_Pin|ON_3_3V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Rele_2_Pin Rele_3_Pin Rele_4_Pin */
  GPIO_InitStruct.Pin = Rele_2_Pin|Rele_3_Pin|Rele_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OE_RELE_Pin */
  GPIO_InitStruct.Pin = OE_RELE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OE_RELE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KTV_ADR_Pin */
  GPIO_InitStruct.Pin = KTV_ADR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KTV_ADR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_KTV_BUF_Pin PCF_INT_Pin */
  GPIO_InitStruct.Pin = PWR_KTV_BUF_Pin|PCF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  on_bkk_k1 = HAL_GPIO_ReadPin(On_BKK_k1_GPIO_Port, On_BKK_k1_Pin);
  on_bkk_k2 = HAL_GPIO_ReadPin(On_BKK_k2_GPIO_Port, On_BKK_k2_Pin);
  error_sv  = HAL_GPIO_ReadPin(error_sv_GPIO_Port, error_sv_Pin);
  door      = HAL_GPIO_ReadPin(DOOR_GPIO_Port, DOOR_Pin);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t on3_3_ones = 0U;
  
  if (htim->Instance == TIM2)
  {
    if (!on3_3_ones)
    {
      if (on_3v3_enabled == 0U)
      {
        tim2_tick_count++;
        if (tim2_tick_count >= ON_3V3_ENABLE_DELAY_TICKS)
        {
          on_3v3_enabled = 1U;
          HAL_GPIO_WritePin(ON_3_3V_GPIO_Port, ON_3_3V_Pin, GPIO_PIN_SET);
          on3_3_ones = 1;
        }
      }
    }
    Ktv_TickISR();
  }
  else if (htim->Instance == TIM1)
  {
    uint8_t sample = 0U;

    /* Снимаем состояние оптронов каждые 1 мс в кольцевой буфер. */
    if (HAL_GPIO_ReadPin(Optron1_1_GPIO_Port, Optron1_1_Pin) == GPIO_PIN_SET)
    {
      sample |= OPTRON_SAMPLE_1_1;
    }
    if (HAL_GPIO_ReadPin(Optron1_2_GPIO_Port, Optron1_2_Pin) == GPIO_PIN_SET)
    {
      sample |= OPTRON_SAMPLE_1_2;
    }
    if (HAL_GPIO_ReadPin(Optron2_1_GPIO_Port, Optron2_1_Pin) == GPIO_PIN_SET)
    {
      sample |= OPTRON_SAMPLE_2_1;
    }
    if (HAL_GPIO_ReadPin(Optron2_2_GPIO_Port, Optron2_2_Pin) == GPIO_PIN_SET)
    {
      sample |= OPTRON_SAMPLE_2_2;
    }

    /* Окно сбора 250 мс, далее 750 мс пауза до следующего цикла. */
    if (optron_cycle_tick < OPTRON_FRAME_SAMPLES)
    {
      optron_samples[optron_sample_index] = sample;
      optron_sample_index++;
      if (optron_sample_index >= OPTRON_FRAME_SAMPLES)
      {
        optron_sample_index = 0U;
      }

      /* По окончании окна разрешаем анализ буфера. */
      if (optron_cycle_tick == (OPTRON_FRAME_SAMPLES - 1U))
      {
        optron_eval_pending = 1U;
      }
    }

    /* Полный цикл 1 секунда: окно 250 мс + пауза 750 мс. */
    optron_cycle_tick++;
    if (optron_cycle_tick >= OPTRON_EVAL_TICKS)
    {
      optron_cycle_tick = 0U;
    }
  }
}



void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (hi2c->Instance != I2C1)
  {
    return;
  }

  active_i2c_address = (uint8_t)AddrMatchCode;

  if (active_i2c_address == I2C_EXPANDER16_ADDR)
  {
    ClearPcfIntLatch();
  }

  if (TransferDirection == I2C_DIRECTION_RECEIVE)
  {
    PrepareExpanderTx(active_i2c_address);
    if (active_i2c_address == I2C_EXPANDER16_ADDR)
    {
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx_buffer, I2C_EXPANDER16_RX_SIZE, I2C_FIRST_AND_LAST_FRAME);
    }
    else if (active_i2c_address == I2C_EXPANDER8_ADDR)
    {
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx_buffer, I2C_EXPANDER8_RX_SIZE, I2C_FIRST_AND_LAST_FRAME);
    }
    else
    {
      HAL_I2C_EnableListen_IT(hi2c);
    }
  }
  else
  {
    if (active_i2c_address == I2C_EXPANDER16_ADDR)
    {
      active_rx_size = I2C_EXPANDER16_RX_SIZE;
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx_buffer, active_rx_size, I2C_FIRST_AND_LAST_FRAME);
    }
    else if (active_i2c_address == I2C_EXPANDER8_ADDR)
    {
      active_rx_size = I2C_EXPANDER8_RX_SIZE;
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx_buffer, active_rx_size, I2C_FIRST_AND_LAST_FRAME);
    }
    else
    {
      HAL_I2C_EnableListen_IT(hi2c);
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1)
  {
    return;
  }

  if (active_i2c_address == I2C_EXPANDER16_ADDR)
  {
    uint16_t value = i2c_rx_buffer[0];
    if (active_rx_size > 1U)
    {
      value |= (uint16_t)(i2c_rx_buffer[1] << 8);
    }
    ApplyExpander16State(value);
  }
  else if (active_i2c_address == I2C_EXPANDER8_ADDR)
  {
    ApplyExpander8State(i2c_rx_buffer[0]);
  }
  
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DOOR_Pin)
  {
    door = HAL_GPIO_ReadPin(DOOR_GPIO_Port, DOOR_Pin);
    return;
  }

  if (GPIO_Pin == error_sv_Pin)
  {
    error_sv = HAL_GPIO_ReadPin(error_sv_GPIO_Port, error_sv_Pin);
    return;
  }

  if (GPIO_Pin == On_BKK_k1_Pin)
  {
    on_bkk_k1 = HAL_GPIO_ReadPin(On_BKK_k1_GPIO_Port, On_BKK_k1_Pin);
    return;
  }

  if (GPIO_Pin == On_BKK_k2_Pin)
  {
    on_bkk_k2 = HAL_GPIO_ReadPin(On_BKK_k2_GPIO_Port, On_BKK_k2_Pin);
    return;
  }

  if (GPIO_Pin != SD_SW_Pin)
  {
    return;
  }

  GPIO_PinState current_state = HAL_GPIO_ReadPin(SD_SW_GPIO_Port, SD_SW_Pin);
  if (current_state == sd_sw_last_state)
  {
    return;
  }

  if (pcf_int_latched == 0U)
  {
    sd_sw_return_state = sd_sw_last_state;
    pcf_int_latched = 1U;
    SetPcfIntLevel(GPIO_PIN_RESET);
  }
  else if (current_state == sd_sw_return_state)
  {
    pcf_int_latched = 0U;
    SetPcfIntLevel(GPIO_PIN_SET);
  }

  sd_sw_last_state = current_state;
}


static inline void EXTI_DisableLine(uint16_t pinmask)
{
  EXTI->IMR &= ~pinmask;   // запретить прерывание по линии
  EXTI->PR   =  pinmask;   // сбросить pending, чтобы не прилетело сразу после enable
}

static inline void EXTI_EnableLine(uint16_t pinmask)
{
  EXTI->PR   =  pinmask;   // сбросить pending
  EXTI->IMR |=  pinmask;   // разрешить прерывания
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
