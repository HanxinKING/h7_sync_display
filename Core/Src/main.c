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
#include "arm_math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM3_CLK_HZ 240000000U
#define TIM6_CLK_HZ 240000000U
#define DAC_SAMPLE_RATE_HZ 1025641U
#define COMP_CAPTURE_CLK_HZ 10000000U
#define COMP_TIM_PRESCALER ((TIM3_CLK_HZ / COMP_CAPTURE_CLK_HZ) - 1U)
#define COMP_TIM_PERIOD 0xFFFFU
#define COMP_MIN_PERIOD_TICKS 50U
#define COMP_MAX_PERIOD_TICKS 11000U
#define COMP_PERIOD_AVG_COUNT 2U
#define COMP_FREQ_SWITCH_CONFIRM_COUNT 4U
#define COMP_TRIM_KI_HZ 0.20f
#define COMP_TRIM_MAX_HZ 8.0f
#define ADC_VREF_V 3.3f
#define ADC_FULL_SCALE_CODE 4095.0f
#define DAC_THRESHOLD_CODE 1861U
#define TWO_PI_F 6.28318530717958647692f
#define INPUT_FREQ_MIN_HZ 1000.0f
#define INPUT_FREQ_MAX_HZ 100000.0f
#define DAC_BUFFER_LEN 256U
#define DAC_HALF_BUFFER_LEN (DAC_BUFFER_LEN / 2U)
#define DAC_LUT_SIZE 256U
#define DAC_LOCK_PHASE_ALPHA 0.50f
#define DAC_OUTPUT_OFFSET_V 1.5f
#define DAC_OUTPUT_AMPLITUDE_V 1.4f
#define UART_REPORT_INTERVAL_MS 50U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
__attribute__((section(".dma_buffer"), aligned(32)))
uint32_t dac_dma_buffer[DAC_BUFFER_LEN];

volatile float32_t dac_output_frequency_hz = INPUT_FREQ_MIN_HZ;
volatile float32_t dac_target_frequency_hz = INPUT_FREQ_MIN_HZ;
volatile float32_t dac_frequency_trim_hz = 0.0f;
volatile float32_t dac_phase_error_cycles = 0.0f;
volatile uint32_t dac_update_rate_hz = DAC_SAMPLE_RATE_HZ;
volatile uint32_t dac_dma_half_count = 0U;
volatile uint32_t dac_dma_full_count = 0U;
volatile uint32_t dac_dma_error_count = 0U;
volatile uint32_t dac_dma_underrun_count = 0U;

volatile uint32_t uart_report_count = 0U;
volatile uint32_t uart_last_report_tick = 0U;

volatile uint32_t comp_input_capture_ticks = 0U;
volatile uint32_t comp_feedback_capture_ticks = 0U;
volatile uint32_t comp_input_period_ticks = 0U;
volatile uint32_t comp_input_frequency_hz = 0U;
volatile uint32_t comp_nominal_frequency_hz = 0U;
volatile uint32_t comp_candidate_frequency_hz = 0U;
volatile uint8_t comp_candidate_count = 0U;
volatile int32_t comp_phase_reference_ticks = 0;
volatile int32_t comp_phase_error_ticks = 0;
volatile uint8_t comp_phase_reference_valid = 0U;
volatile float32_t comp_trim_hz = 0.0f;
volatile uint32_t comp_lock_valid = 0U;

uint16_t dac_sine_lut[DAC_LUT_SIZE];
volatile uint32_t dac_phase_step = 0U;
volatile uint32_t dac_phase_accumulator = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART5_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
/* USER CODE BEGIN PFP */
static void InitDacGenerator(void);
static void StartDacTimerDma(void);
static void FillDacBuffer(uint32_t offset, uint32_t length);
static uint32_t FrequencyToPhaseStep(float32_t frequency_hz);
static int32_t WrapPhaseErrorTicks(int32_t diff_ticks, uint32_t period_ticks);
static void UpdateComparatorLockOnInputEdge(uint32_t capture_ticks);
static void SendStartupBannerUart5(void);
static void ReportLockStatusUart5(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void InitDacGenerator(void)
{
  uint32_t i;
  float32_t phase;
  float32_t offset_code = (DAC_OUTPUT_OFFSET_V * ADC_FULL_SCALE_CODE) / ADC_VREF_V;
  float32_t amplitude_code = (DAC_OUTPUT_AMPLITUDE_V * ADC_FULL_SCALE_CODE) / ADC_VREF_V;

  if (amplitude_code > offset_code)
  {
    amplitude_code = offset_code;
  }

  for (i = 0U; i < DAC_LUT_SIZE; i++)
  {
    phase = TWO_PI_F * (float32_t)i / (float32_t)DAC_LUT_SIZE;
    dac_sine_lut[i] = (uint16_t)(offset_code + amplitude_code * arm_sin_f32(phase));
  }

  dac_output_frequency_hz = INPUT_FREQ_MIN_HZ;
  dac_target_frequency_hz = INPUT_FREQ_MIN_HZ;
  dac_frequency_trim_hz = 0.0f;
  dac_phase_error_cycles = 0.0f;
  dac_phase_step = FrequencyToPhaseStep(dac_output_frequency_hz);
  dac_phase_accumulator = 0U;
  FillDacBuffer(0U, DAC_BUFFER_LEN);
}

static void StartDacTimerDma(void)
{
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_dma_buffer, DAC_BUFFER_LEN, DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
}

static void FillDacBuffer(uint32_t offset, uint32_t length)
{
  uint32_t i;
  uint32_t local_phase;
  uint32_t local_step;

  __disable_irq();
  local_phase = dac_phase_accumulator;
  local_step = dac_phase_step;
  __enable_irq();

  for (i = 0U; i < length; i++)
  {
    dac_dma_buffer[offset + i] = dac_sine_lut[(local_phase >> 24) & 0xFFU];
    local_phase += local_step;
  }

  __disable_irq();
  dac_phase_accumulator = local_phase;
  __enable_irq();
}

static uint32_t FrequencyToPhaseStep(float32_t frequency_hz)
{
  float64_t step;

  if (frequency_hz < INPUT_FREQ_MIN_HZ)
  {
    frequency_hz = INPUT_FREQ_MIN_HZ;
  }
  else if (frequency_hz > INPUT_FREQ_MAX_HZ)
  {
    frequency_hz = INPUT_FREQ_MAX_HZ;
  }

  step = ((float64_t)frequency_hz * 4294967296.0) / (float64_t)DAC_SAMPLE_RATE_HZ;
  if (step < 1.0)
  {
    step = 1.0;
  }

  return (uint32_t)step;
}

static int32_t WrapPhaseErrorTicks(int32_t diff_ticks, uint32_t period_ticks)
{
  int32_t half_period;

  if (period_ticks == 0U)
  {
    return diff_ticks;
  }

  half_period = (int32_t)(period_ticks / 2U);
  while (diff_ticks > half_period)
  {
    diff_ticks -= (int32_t)period_ticks;
  }
  while (diff_ticks < -half_period)
  {
    diff_ticks += (int32_t)period_ticks;
  }

  return diff_ticks;
}

static void UpdateComparatorLockOnInputEdge(uint32_t capture_ticks)
{
  static uint32_t prev_capture_ticks = 0U;
  static uint8_t prev_valid = 0U;
  static uint32_t period_hist[COMP_PERIOD_AVG_COUNT] = {0U};
  static uint32_t period_hist_sum = 0U;
  static uint32_t period_hist_index = 0U;
  static uint32_t period_hist_count = 0U;
  uint32_t period_ticks;
  uint32_t measured_hz;
  uint32_t quantized_hz;
  uint32_t new_phase_step;
  int32_t diff_ticks;
  int32_t phase_adjust;
  float32_t phase_error_cycles;
  float32_t trim_hz;

  comp_input_capture_ticks = capture_ticks;

  if (prev_valid == 0U)
  {
    prev_capture_ticks = capture_ticks;
    prev_valid = 1U;
    return;
  }

  period_ticks = (uint32_t)((capture_ticks - prev_capture_ticks) & COMP_TIM_PERIOD);
  prev_capture_ticks = capture_ticks;

  if ((period_ticks < COMP_MIN_PERIOD_TICKS) || (period_ticks > COMP_MAX_PERIOD_TICKS))
  {
    comp_lock_valid = 0U;
    return;
  }

  period_hist_sum -= period_hist[period_hist_index];
  period_hist[period_hist_index] = period_ticks;
  period_hist_sum += period_ticks;
  period_hist_index++;
  if (period_hist_index >= COMP_PERIOD_AVG_COUNT)
  {
    period_hist_index = 0U;
  }
  if (period_hist_count < COMP_PERIOD_AVG_COUNT)
  {
    period_hist_count++;
  }

  period_ticks = period_hist_sum / period_hist_count;

  comp_input_period_ticks = period_ticks;
  measured_hz = COMP_CAPTURE_CLK_HZ / period_ticks;
  comp_input_frequency_hz = measured_hz;
  quantized_hz = ((measured_hz + 500U) / 1000U) * 1000U;

  if (comp_nominal_frequency_hz == 0U)
  {
    comp_nominal_frequency_hz = quantized_hz;
    comp_candidate_frequency_hz = quantized_hz;
    comp_candidate_count = 0U;
    comp_phase_reference_valid = 0U;
    comp_trim_hz = 0.0f;
  }
  else if (quantized_hz == comp_nominal_frequency_hz)
  {
    comp_candidate_frequency_hz = quantized_hz;
    comp_candidate_count = 0U;
  }
  else
  {
    if (quantized_hz == comp_candidate_frequency_hz)
    {
      if (comp_candidate_count < 255U)
      {
        comp_candidate_count++;
      }
    }
    else
    {
      comp_candidate_frequency_hz = quantized_hz;
      comp_candidate_count = 1U;
    }

    if (comp_candidate_count >= COMP_FREQ_SWITCH_CONFIRM_COUNT)
    {
      comp_nominal_frequency_hz = comp_candidate_frequency_hz;
      comp_candidate_count = 0U;
      comp_phase_reference_valid = 0U;
      comp_trim_hz = 0.0f;
      dac_frequency_trim_hz = 0.0f;
    }
  }

  dac_target_frequency_hz = (float32_t)comp_nominal_frequency_hz;
  if ((comp_nominal_frequency_hz < (uint32_t)INPUT_FREQ_MIN_HZ) || (comp_nominal_frequency_hz > (uint32_t)INPUT_FREQ_MAX_HZ))
  {
    comp_lock_valid = 0U;
    return;
  }

  if (comp_phase_reference_valid == 0U)
  {
    diff_ticks = WrapPhaseErrorTicks((int32_t)comp_feedback_capture_ticks - (int32_t)capture_ticks, period_ticks);
    comp_phase_reference_ticks = diff_ticks;
    comp_phase_reference_valid = 1U;
    comp_phase_error_ticks = 0;
    comp_lock_valid = 0U;
    dac_output_frequency_hz = dac_target_frequency_hz;
    comp_trim_hz = 0.0f;
    dac_frequency_trim_hz = 0.0f;
    new_phase_step = FrequencyToPhaseStep(dac_output_frequency_hz);
    __disable_irq();
    dac_phase_step = new_phase_step;
    __enable_irq();
    return;
  }

  diff_ticks = WrapPhaseErrorTicks((int32_t)comp_feedback_capture_ticks - (int32_t)capture_ticks, period_ticks);
  diff_ticks = WrapPhaseErrorTicks(diff_ticks - comp_phase_reference_ticks, period_ticks);
  comp_phase_error_ticks = diff_ticks;

  phase_error_cycles = (float32_t)diff_ticks / (float32_t)period_ticks;
  dac_phase_error_cycles = phase_error_cycles;

  trim_hz = comp_trim_hz + phase_error_cycles * COMP_TRIM_KI_HZ;
  if (trim_hz > COMP_TRIM_MAX_HZ)
  {
    trim_hz = COMP_TRIM_MAX_HZ;
  }
  else if (trim_hz < -COMP_TRIM_MAX_HZ)
  {
    trim_hz = -COMP_TRIM_MAX_HZ;
  }
  comp_trim_hz = trim_hz;
  dac_frequency_trim_hz = trim_hz;

  dac_output_frequency_hz = dac_target_frequency_hz + comp_trim_hz;
  new_phase_step = FrequencyToPhaseStep(dac_output_frequency_hz);
  __disable_irq();
  dac_phase_step = new_phase_step;
  __enable_irq();

  phase_adjust = (int32_t)(phase_error_cycles * 4294967296.0f * DAC_LOCK_PHASE_ALPHA * 0.1f);
  __disable_irq();
  dac_phase_accumulator = (uint32_t)((int32_t)dac_phase_accumulator + phase_adjust);
  __enable_irq();

  comp_lock_valid = 1U;
}

static void SendStartupBannerUart5(void)
{
  static const char banner[] =
      "\r\nUART5 alive v6-compcap | comp/tim lock | input->PB0 | dacfb->PE9 | wire PA4->PE9\r\n";

  (void)HAL_UART_Transmit(&huart5, (uint8_t *)banner, (uint16_t)(sizeof(banner) - 1U), 100U);
}

static void ReportLockStatusUart5(void)
{
  char tx_buf[192];
  int len;
  uint32_t now;

  now = HAL_GetTick();
  if ((now - uart_last_report_tick) < UART_REPORT_INTERVAL_MS)
  {
    return;
  }

  uart_last_report_tick = now;
  len = snprintf(tx_buf,
                 sizeof(tx_buf),
                 "tick=%lu,fin=%lu,fnom=%lu,fcand=%lu,fcnt=%u,dac_t_mhz=%ld,trim_mhz=%ld,dac_o_mhz=%ld,per=%lu,phi=%ld,ref=%ld,lock=%lu,dac_h=%lu,dac_f=%lu,dac_e=%lu,dac_u=%lu\r\n",
                 now,
                 comp_input_frequency_hz,
                 comp_nominal_frequency_hz,
                 comp_candidate_frequency_hz,
                 comp_candidate_count,
                 (long)(dac_target_frequency_hz * 1000.0f),
                 (long)(comp_trim_hz * 1000.0f),
                 (long)(dac_output_frequency_hz * 1000.0f),
                 comp_input_period_ticks,
                 (long)comp_phase_error_ticks,
                 (long)comp_phase_reference_ticks,
                 comp_lock_valid,
                 dac_dma_half_count,
                 dac_dma_full_count,
                 dac_dma_error_count,
                 dac_dma_underrun_count);
  if (len > 0)
  {
    if (HAL_UART_Transmit(&huart5, (uint8_t *)tx_buf, (uint16_t)len, 20U) == HAL_OK)
    {
      uart_report_count++;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  MPU_Config();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();

  InitDacGenerator();

  __HAL_DBGMCU_FREEZE_TIM2();
  __HAL_DBGMCU_FREEZE_TIM3();
  __HAL_DBGMCU_FREEZE_TIM6();
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);

  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  StartDacTimerDma();
  SendStartupBannerUart5();

  while (1)
  {
    ReportLockStatusUart5();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};
  DAC_ChannelConfTypeDef sThresholdConfig = {0};

  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  sConfig.DAC_TrimmingValue = 0;
  sConfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 0;
  sConfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 0;
  sConfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 0;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sThresholdConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sThresholdConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sThresholdConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sThresholdConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sThresholdConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  sThresholdConfig.DAC_TrimmingValue = 0;
  sThresholdConfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 0;
  sThresholdConfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 0;
  sThresholdConfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 0;
  if (HAL_DAC_ConfigChannel(&hdac1, &sThresholdConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_THRESHOLD_CODE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = COMP_TIM_PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = COMP_TIM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 2;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_TISelection(&htim2, TIM_TIM2_TI4_COMP2, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = COMP_TIM_PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = COMP_TIM_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 2;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_TISelection(&htim3, TIM_TIM3_TI1_COMP1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (TIM6_CLK_HZ / DAC_SAMPLE_RATE_HZ) - 1U;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{
  hcomp1.Instance = COMP1;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH2;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_HIGH;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_COMP_Start(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{
  hcomp2.Instance = COMP2;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH2;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_HIGH;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_COMP_Start(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac_dma_half_count++;
    FillDacBuffer(0U, DAC_HALF_BUFFER_LEN);
  }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac_dma_full_count++;
    FillDacBuffer(DAC_HALF_BUFFER_LEN, DAC_HALF_BUFFER_LEN);
  }
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  (void)hdac;
  dac_dma_error_count++;
}

void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
  (void)hdac;
  dac_dma_underrun_count++;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    UpdateComparatorLockOnInputEdge(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  }
  else if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
  {
    comp_feedback_capture_ticks = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
  }
}
/* USER CODE END 4 */

/* MPU Configuration */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
