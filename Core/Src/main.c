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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "acs712.h"
#include "pwm_driver.h"
#include "pid.h"
#include "usb_comm.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float T_start;
    float T_target;
    float T_boost;
    uint8_t is_boosting;
} Target_Boost_HandleTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE            64     
#define TJ_POD_MAX_CURRENT_A    2.7f   
#define TJ_POD_MAX_TORQUE_NM    200.0f
#define TORQUE_TO_CURRENT_KT    74.07f
#define CONTROL_LOOP_TIME_MS    1
#define TELEMETRY_TIME_MS       20
#define BRAKE_TAU_SEC           0.37f
#define DESIRED_STEP_TIME_SEC   0.40f
#define COIL_RESISTANCE_OHM     8.0f
#define SUPPLY_VOLTAGE_V        24.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint16_t adc1_dma_buf[ADC_BUF_SIZE];

ACS712_HandleTypeDef       h_acs712;
PWM_Driver_HandleTypeDef   h_brake_pwm;
PID_HandleTypeDef          h_pid_current;
Target_Boost_HandleTypeDef h_boost = {0.0f, 0.0f, 0.0f, 0};

volatile float g_measured_current_A  = 0.0f;
volatile float g_measured_current_mA = 0.0f;
volatile float g_real_torque_Nm      = 0.0f;
volatile float g_target_current_A    = 0.0f;
volatile float g_pwm_duty_output     = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
 HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buf, ADC_BUF_SIZE);

  // 2. Khởi tạo Driver cảm biến ACS712 chuẩn khớp theo acs712.h
  // Tham số: (dev, hadc, dma_buf, buf_size)
  ACS712_Init(&h_acs712, &hadc1, adc1_dma_buf, ADC_BUF_SIZE);

  // 3. Hiệu chỉnh điểm Zero khi chưa đóng tải
  ACS712_CalibrateZero(&h_acs712);

  // 4. Khởi tạo Driver PWM cho Module XY-MOSFET (TIM1_CH1)
  PWM_Driver_Init(&h_brake_pwm, &htim1, TIM_CHANNEL_1);
  PWM_Driver_Start(&h_brake_pwm);

  // 5. Khởi tạo PID Dòng điện (Kp=15.0, Ki=80.0, Kd=0.0, dt=0.001s, out: 0-100%)
  PID_Init(&h_pid_current, 30.0f, 15.0f, 0.0f, 0.001f, 0.0f, 100.0f);

  // 6. Khởi tạo Module USB CDC
  USB_Comm_Init();

  uint32_t last_control_time = HAL_GetTick();
  uint32_t last_telemetry_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();

    // =========================================================================
// VÒNG LẶP ĐIỀU KHIỂN KÍN 1kHz (1ms) - XỬ LÝ AN TOÀN TRẦN 200Nm & DUMP 0Nm
// =========================================================================
    if (current_time - last_control_time >= CONTROL_LOOP_TIME_MS){
     last_control_time = current_time;

     // 1. Đọc dòng thực tế từ ACS712 và tính mô-men hiện tại
     g_measured_current_A = ACS712_Process(&h_acs712);
     g_measured_current_mA = g_measured_current_A * 1000.0f;
     g_real_torque_Nm = g_measured_current_A * TORQUE_TO_CURRENT_KT;

     // 2. KỊCH BẢN XẢ VỀ 0: Set T = 0 Nm -> Khóa PWM ngay lập tức
     if (g_target_torque_Nm <= 0.0f) {
        PWM_Driver_SetDutyCycle(&h_brake_pwm, 0.0f);
        PID_Reset(&h_pid_current);
        h_boost.is_boosting = 0;
        g_pwm_duty_output = 0.0f;
      }
    else {
        // 3. Phát hiện bước nhảy mô-men đặt mới
        if (fabsf(g_target_torque_Nm - h_boost.T_target) > 1.0f) {
            h_boost.T_start = g_real_torque_Nm;
            h_boost.T_target = g_target_torque_Nm;
            
            // Tính toán T_boost ảo (Hàm này tự động Clamping tối đa 200 Nm)
            h_boost.T_boost = Calculate_Boost_Torque(h_boost.T_start, h_boost.T_target, 0.40f, 0.37f);
            h_boost.is_boosting = 1;
        }

        // 4. CHỐNG VỌT LỐ: Giám sát điểm cắt (Cutoff Threshold)
        float active_target_Nm = h_boost.T_target;

        if (h_boost.is_boosting) {
            active_target_Nm = h_boost.T_boost; 

            // Nếu target = 200Nm hoặc T_boost bị kẹp ở 200Nm: 
            // Ngắt Boost ngay khi dòng đo đạt 96% của Target thật (tránh vọt lố)
            if ((h_boost.T_target > h_boost.T_start && g_real_torque_Nm >= h_boost.T_target * 0.96f) ||
                (h_boost.T_target < h_boost.T_start && g_real_torque_Nm <= h_boost.T_target * 1.04f)) 
            {
                h_boost.is_boosting = 0; // Tắt Boost, trả lại cho PID
                active_target_Nm = h_boost.T_target;
                
                // Pre-load tích phân PID bằng điện áp duy trì xác lập chuẩn
                float target_I = h_boost.T_target / TORQUE_TO_CURRENT_KT;
                h_pid_current.integrator = (target_I * COIL_RESISTANCE_OHM / SUPPLY_VOLTAGE_V) * 100.0f;
            }
        }

        // 5. Tính toán dòng đặt cho PID
        g_target_current_A = active_target_Nm / TORQUE_TO_CURRENT_KT;
        if (g_target_current_A > TJ_POD_MAX_CURRENT_A) g_target_current_A = TJ_POD_MAX_CURRENT_A;

        // 6. Tính toán PID & Xuất Duty PWM
        g_pwm_duty_output = PID_Compute(&h_pid_current, g_target_current_A, g_measured_current_A);
        PWM_Driver_SetDutyCycle(&h_brake_pwm, g_pwm_duty_output);
      }
    }

    // =========================================================================
    // KHỐI 2: TRUYỀN DỮ LIỆU TELEMETRY VỀ PC / EXCEL (Tần số 50Hz - Chu kỳ 20ms)
    // =========================================================================
    if (current_time - last_telemetry_time >= TELEMETRY_TIME_MS)
    {
      last_telemetry_time = current_time;

      // Tính Mô-men thực tế phản hồi từ dòng điện đo được
      g_real_torque_Nm = g_measured_current_A * TORQUE_TO_CURRENT_KT;

      // Đẩy chuỗi dữ liệu CSV (Current_mA, Torque_Nm, Duty_Percent) qua USB Type-C
      USB_Comm_SendTelemetry(g_measured_current_mA, g_real_torque_Nm, g_pwm_duty_output);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
