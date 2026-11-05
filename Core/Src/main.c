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
#include "hx711.h"     /* DÒNG NÀY ĐÃ ĐƯỢC BỔ SUNG ĐỂ SỬA LỖI COMPILER */
#include "encoder.h"
#include "usb_comm.h"
#include "road_load.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float T_start;
    float T_target;
    float T_boost;
    uint8_t is_boosting;
    uint32_t start_tick;
} Target_Boost_HandleTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Đặt = 1 để chạy Tool Calib | Đặt = 0 để chạy chương trình chính */
#define CALIBRATION_MODE        0
/* Chọn cảm biến cần Calib: 1 = Loadcell 1 (Nén M+), 2 = Loadcell 2 (Kéo M-) */
#define CALIB_SELECT_SENSOR     1
/* Đổi khối lượng quả cân chuẩn của ở đây (Ví dụ: 7.425 kg hoặc 1.0 kg) */
#define KNOWN_WEIGHT_KG         7.425f
#define ADC_BUF_SIZE            64     
#define TJ_POD_MAX_CURRENT_A    2.7f   
#define TJ_POD_MAX_TORQUE_NM    200.0f
#define TORQUE_TO_CURRENT_KT    74.07f
#define CONTROL_LOOP_TIME_MS    1
#define TELEMETRY_TIME_MS       20
#define BRAKE_TAU_SEC           0.37f
#define DESIRED_STEP_TIME_SEC   0.05f
#define COIL_RESISTANCE_OHM     8.0f
#define SUPPLY_VOLTAGE_V        24.0f
/* Thời gian boost tối đa, tránh kẹt boost khi nhiễu dòng không cho đạt ngưỡng thoát */
#define BOOST_MAX_TIME_MS       250
/* Failsafe: quá thời gian này không nhận lệnh => nhả phanh. 0 = tắt
 * (Khi bật, phần mềm PC phải gửi lại lệnh T định kỳ) */
#define CMD_TIMEOUT_MS          0

/* Cánh tay đòn: khoảng cách từ tâm cảm biến lực đến tâm phanh bột từ = 198 mm
 * Mô-men phanh đo được: T = F_loadcell * L */
#define DYNO_ARM_1_LENGTH_M     0.198f /* Cánh tay đòn Loadcell 1 (mét) */
#define DYNO_ARM_2_LENGTH_M     0.198f /* Cánh tay đòn Loadcell 2 (mét) */
#define DYNO_ARM_LENGTH_M       0.198f /* Tương thích ngược */

/* Vòng kín mô-men (vòng ngoài): so mô-men đo từ loadcell với giá trị đặt,
 * cộng phần hiệu chỉnh vào mô-men đặt cho vòng dòng (vòng trong).
 * T_cmd = T_set + PI(T_ref - T_loadcell)
 * T_ref: mô-men kỳ vọng = T_set qua khâu trễ bậc 1 (trễ từ phanh + lọc loadcell),
 * để PI chỉ bù sai số xác lập (Kt lệch, nhiệt độ...) mà không bị vọt lố do trễ */
#define TORQUE_LOOP_KP          0.5f
#define TORQUE_LOOP_KI          4.0f   /* 1/s */
#define TORQUE_LOOP_LC_TAU_SEC  0.12f  /* Trễ do lọc EMA loadcell (~80 SPS, alpha 0.1) */
#define TORQUE_LOOP_CORR_MAX_NM 50.0f  /* Giới hạn phần hiệu chỉnh */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint16_t adc1_dma_buf[ADC_BUF_SIZE];

ACS712_HandleTypeDef       h_acs712;
PWM_Driver_HandleTypeDef   h_brake_pwm;
PID_HandleTypeDef          h_pid_current;
PID_HandleTypeDef          h_pid_torque;   /* Vòng ngoài: mô-men loadcell */
HX711_HandleTypeDef        h_hx711_1;  /* Loadcell 1 (Bên nén M+) - PA4/PA5 */
HX711_HandleTypeDef        h_hx711_2;  /* Loadcell 2 (Bên kéo M-) - PB0/PB1 */
Encoder_HandleTypeDef      h_encoder;
RoadLoad_HandleTypeDef     h_road_load;
Target_Boost_HandleTypeDef h_boost = {0.0f, 0.0f, 0.0f, 0, 0};

volatile float g_measured_current_A      = 0.0f;
volatile float g_measured_current_mA     = 0.0f;
volatile float g_torque_from_current_Nm  = 0.0f; 
volatile float g_torque_from_loadcell_Nm = 0.0f; /* Tổng mô-men Dyno: |M1| + |M2| (Tự động chống ngược) */
volatile float g_torque_loadcell_1_Nm    = 0.0f; /* Mô-men nhánh 1 */
volatile float g_torque_loadcell_2_Nm    = 0.0f; /* Mô-men nhánh 2 */
volatile float g_measured_speed_rpm      = 0.0f;
volatile float g_mechanical_power_w      = 0.0f;
volatile float g_target_current_A        = 0.0f;
volatile float g_pwm_duty_output         = 0.0f;
volatile float g_torque_ref_Nm           = 0.0f; /* Mô-men kỳ vọng cho vòng kín loadcell */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Run_HX711_Calibration_Tool(HX711_HandleTypeDef *hx, uint8_t sensor_id) {
    static char msg[128];

    // 1. Lấy mốc Zero khi chưa đặt tải
    snprintf(msg, sizeof(msg), "\r\n=== HE THONG CALIB LOADCELL %d ===\r\n1. Dang lay moc Zero (Khong dat gi len phanh)...", sensor_id);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    HX711_Tare(hx, 40);
    
    snprintf(msg, sizeof(msg), " XONG!\r\n -> ZERO OFFSET: %ld\r\n", hx->offset);
    HAL_Delay(50);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(2000);

    // 2. Yêu cầu đặt quả cân lên
    char str_weight[16];
    Format_Float(str_weight, sizeof(str_weight), KNOWN_WEIGHT_KG, 2);
    snprintf(msg, sizeof(msg), "2. Hay treo/dat qua can %s kg len Loadcell %d trong 5 giay...\r\n", str_weight, sensor_id);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    
    for (int i = 5; i > 0; i--) {
        snprintf(msg, sizeof(msg), " Dem nguoc: %d s\r\n", i);
        HAL_Delay(50);
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        HAL_Delay(4000);
    }

    // 3. Đọc mẫu có tải để tính Scale Factor
    snprintf(msg, sizeof(msg), "3. Dang tinh toan Scale Factor Loadcell %d...", sensor_id);
    HAL_Delay(50);
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

    int64_t sum = 0;
    uint8_t count = 0;
    uint32_t step3_timeout = HAL_GetTick();

    while (count < 40) {
        if (HX711_IsReady(hx)) {
            sum += HX711_ReadRaw(hx);
            count++;
        }
        /* Bảo vệ chống treo nếu cảm biến tuột dây */
        if (HAL_GetTick() - step3_timeout > 6000) {
            snprintf(msg, sizeof(msg), "\r\n[LOI]: Timeout doc HX711! Kiem tra day ket noi.\r\n");
            CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            return;
        }
    }
    int32_t avg_raw = (int32_t)(sum / 40);
    float calculated_scale = (float)(avg_raw - hx->offset) / KNOWN_WEIGHT_KG;
    char str_scale[24];
    Format_Float(str_scale, sizeof(str_scale), calculated_scale, 2);

    // 4. In kết quả cuối cùng
    static char big_buffer[384]; /* Dùng static và 384 bytes an toàn cho RAM */
    snprintf(big_buffer, sizeof(big_buffer),
             " XONG!\r\n\r\n=========================================\r\n"
             "KET QUA CALIB HOAN TAT CHO LOADCELL %d:\r\n"
             "-> ZERO OFFSET: %ld\r\n"
             "-> RAW CO TAI (AVG): %ld\r\n"
             "-> DELTA ADC (LOAD - ZERO): %ld\r\n"
             "-> SCALE FACTOR: %s\r\n\r\n"
             "-> HAY COPY DONG SAU VAO FILE hx711.h:\r\n"
             "#define HX711_%d_SCALE_FACTOR  %sf\r\n"
             "=========================================\r\n",
             sensor_id, (long)hx->offset, (long)avg_raw, (long)(avg_raw - hx->offset), str_scale,
             sensor_id, str_scale);

    HAL_Delay(100);
    CDC_Transmit_FS((uint8_t*)big_buffer, strlen(big_buffer));
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_Device_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buf, ADC_BUF_SIZE);

  ACS712_Init(&h_acs712, &hadc1, adc1_dma_buf, ADC_BUF_SIZE);
  ACS712_CalibrateZero(&h_acs712);

  PWM_Driver_Init(&h_brake_pwm, &htim1, TIM_CHANNEL_1);
  PWM_Driver_Start(&h_brake_pwm);

  PID_Init(&h_pid_current, 30.0f, 15.0f, 0.0f, 0.001f, 0.0f, 100.0f);
  PID_Init(&h_pid_torque, TORQUE_LOOP_KP, TORQUE_LOOP_KI, 0.0f, 0.001f,
           -TORQUE_LOOP_CORR_MAX_NM, TORQUE_LOOP_CORR_MAX_NM);

  // Khởi tạo HX711 #1 (Loadcell 1 bên nén M+: DOUT=PA4, SCK=PA5)
  HX711_Init(&h_hx711_1, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5, DYNO_ARM_1_LENGTH_M);
  h_hx711_1.scale_factor = HX711_1_SCALE_FACTOR;

  // Khởi tạo HX711 #2 (Loadcell 2 bên kéo M-: DOUT=PB0, SCK=PB1)
  HX711_Init(&h_hx711_2, GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, DYNO_ARM_2_LENGTH_M);
  h_hx711_2.scale_factor = HX711_2_SCALE_FACTOR;

  // Khởi tạo Encoder đo tốc độ trục phanh (PPR = 2000, 4X = 8000 counts/vòng trên TIM3 PA6/PB5)
  Encoder_Init(&h_encoder, &htim3, 2000.0f);
  Encoder_Start(&h_encoder);

  RoadLoad_Init(&h_road_load, ROAD_LOAD_DEFAULT_MASS_KG, ROAD_LOAD_DEFAULT_RADIUS_M);

  USB_Comm_Init();

#if CALIBRATION_MODE
  HAL_Delay(5000);                      
#if (CALIB_SELECT_SENSOR == 2)
  Run_HX711_Calibration_Tool(&h_hx711_2, 2);
#else
  Run_HX711_Calibration_Tool(&h_hx711_1, 1);
#endif
  while(1);               
#else
  HX711_Tare(&h_hx711_1, 20);
  HX711_Tare(&h_hx711_2, 20);
#endif

  uint32_t last_cmd_time = HAL_GetTick();
  (void)last_cmd_time; /* Chỉ dùng khi CMD_TIMEOUT_MS > 0 */
  uint32_t last_control_time = HAL_GetTick();
  uint32_t last_telemetry_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();

    // =========================================================================
    // VÒNG LẶP ĐIỀU KHIỂN KÍN 1kHz (1ms)
    // =========================================================================
    if (current_time - last_control_time >= CONTROL_LOOP_TIME_MS) {
        last_control_time = current_time;

        /* Dòng lọc nhanh cho PID, dòng lọc mượt cho hiển thị */
        g_measured_current_A = ACS712_Process(&h_acs712);
        ACS712_AutoZero(&h_acs712, (g_pwm_duty_output <= 0.0f) ? 1 : 0);
        g_measured_current_mA = ACS712_GetDisplayCurrent(&h_acs712) * 1000.0f;
        g_torque_from_current_Nm = g_measured_current_A * TORQUE_TO_CURRENT_KT;

        HX711_Process(&h_hx711_1);
        HX711_Process(&h_hx711_2);
        g_torque_loadcell_1_Nm = h_hx711_1.measured_torque_nm;
        g_torque_loadcell_2_Nm = h_hx711_2.measured_torque_nm;

        /* Tổng mô-men Dyno từ 2 loadcell đối xứng (Tự động chống ngược chiều):
         * Sử dụng tổng biên độ lực cản: M_total = |M1| + |M2|
         * - Bất chấp cắm Loadcell nào vào kênh 1 hay kênh 2.
         * - Bất chấp động cơ quay thuận hay quay nghịch.
         * - Nếu chỉ cắm 1 Loadcell (kênh còn lại rút dây), kênh đó = 0 => M_total = |M1|.
         */
        g_torque_from_loadcell_Nm = fabsf(g_torque_loadcell_1_Nm) + fabsf(g_torque_loadcell_2_Nm);

        /* Failsafe: mất kết nối USB (hoặc quá hạn lệnh) => nhả phanh */
        if (g_cmd_received_flag) {
            g_cmd_received_flag = 0;
            last_cmd_time = current_time;
        }
        if (!USB_Comm_IsConnected()
#if (CMD_TIMEOUT_MS > 0)
            || (current_time - last_cmd_time > CMD_TIMEOUT_MS)
#endif
           ) {
            g_target_torque_Nm = 0.0f;
        }

        if (g_target_torque_Nm <= 0.0f) {
            PWM_Driver_SetDutyCycle(&h_brake_pwm, 0.0f);
            PID_Reset(&h_pid_current);
            PID_Reset(&h_pid_torque);
            g_torque_ref_Nm = fabsf(g_torque_from_loadcell_Nm);
            h_boost.is_boosting = 0;
            h_boost.T_target = 0.0f;
            g_pwm_duty_output = 0.0f;
        } 
        else {
            /* Chế độ mô phỏng: mô-men thay đổi liên tục => không dùng boost bước nhảy */
            if (!g_road_sim_enabled && fabsf(g_target_torque_Nm - h_boost.T_target) > 1.0f) {
                h_boost.T_start = g_torque_from_current_Nm;
                h_boost.T_target = g_target_torque_Nm;
                h_boost.T_boost = Calculate_Boost_Torque(h_boost.T_start, h_boost.T_target, DESIRED_STEP_TIME_SEC, BRAKE_TAU_SEC);
                h_boost.is_boosting = 1;
                h_boost.start_tick = current_time;
            } else {
              h_boost.T_target = g_target_torque_Nm;
              if (g_road_sim_enabled) h_boost.is_boosting = 0;
            }

            float active_target_Nm = h_boost.T_target;

            if (h_boost.is_boosting) {
                active_target_Nm = h_boost.T_boost; 

                if ((h_boost.T_target > h_boost.T_start && g_torque_from_current_Nm >= h_boost.T_target * 0.96f) ||
                    (h_boost.T_target < h_boost.T_start && g_torque_from_current_Nm <= h_boost.T_target * 1.04f) ||
                    (current_time - h_boost.start_tick > BOOST_MAX_TIME_MS))
                {
                    h_boost.is_boosting = 0;
                    active_target_Nm = h_boost.T_target;
                    
                    float target_I = h_boost.T_target / TORQUE_TO_CURRENT_KT;
                    h_pid_current.integrator = (target_I * COIL_RESISTANCE_OHM / SUPPLY_VOLTAGE_V) * 100.0f;
                }
            }

            /* Vòng kín mô-men theo loadcell: chỉ chạy khi hết boost và có cảm biến */
            float T_meas_Nm = fabsf(g_torque_from_loadcell_Nm);
            if (!h_boost.is_boosting && (h_hx711_1.is_connected || h_hx711_2.is_connected)) {
                g_torque_ref_Nm += (h_boost.T_target - g_torque_ref_Nm) *
                                 ((CONTROL_LOOP_TIME_MS * 0.001f) / (BRAKE_TAU_SEC + TORQUE_LOOP_LC_TAU_SEC));
                float corr_Nm = PID_Compute(&h_pid_torque, g_torque_ref_Nm, T_meas_Nm);
                active_target_Nm = h_boost.T_target + corr_Nm;
                if (active_target_Nm < 0.0f) active_target_Nm = 0.0f;
            } else {
                /* Bắt đầu mô hình kỳ vọng từ giá trị đo thực tế */
                g_torque_ref_Nm = T_meas_Nm;
                PID_Reset(&h_pid_torque);
            }

            g_target_current_A = active_target_Nm / TORQUE_TO_CURRENT_KT;
            if (g_target_current_A > TJ_POD_MAX_CURRENT_A) g_target_current_A = TJ_POD_MAX_CURRENT_A;

            g_pwm_duty_output = PID_Compute(&h_pid_current, g_target_current_A, g_measured_current_A);
            PWM_Driver_SetDutyCycle(&h_brake_pwm, g_pwm_duty_output);
        }
    }

    // =========================================================================
    // KHỐI TELEMETRY 50Hz (20ms) - CẬP NHẬT TỐC ĐỘ ENCODER & GỬI DỮ LIỆU
    // =========================================================================
    if (current_time - last_telemetry_time >= TELEMETRY_TIME_MS) {
        uint32_t dt_ms = current_time - last_telemetry_time;
        last_telemetry_time = current_time;

        /* Tính toán tốc độ quay RPM và Công suất cơ học */
        float active_torque = (fabsf(g_torque_from_loadcell_Nm) > 0.01f) ? fabsf(g_torque_from_loadcell_Nm) : g_torque_from_current_Nm;
        Encoder_Process(&h_encoder, (float)dt_ms, active_torque);

        g_mechanical_power_w = Encoder_GetPower(&h_encoder);

        /* Mô phỏng tải đường: Fw = m*a + 0.088*m + (0.000015*m + 0.02)*v_kmh^2 */
        RoadLoad_SetParams(&h_road_load, g_vehicle_mass_kg, g_wheel_radius_m);
        float road_torque_Nm = RoadLoad_Process(&h_road_load, Encoder_GetOmega(&h_encoder), (float)dt_ms / 1000.0f);

        /* Tốc độ quay bánh xe = v / r */
        g_measured_speed_rpm = h_road_load.wheel_rpm;
        if (g_road_sim_enabled && USB_Comm_IsConnected()) {
            if (road_torque_Nm > TJ_POD_MAX_TORQUE_NM) road_torque_Nm = TJ_POD_MAX_TORQUE_NM;
            g_target_torque_Nm = road_torque_Nm;
        }

        USB_Comm_SendTelemetry(g_measured_current_mA,
                               g_measured_current_mA * 0.001f * TORQUE_TO_CURRENT_KT,
                               g_torque_from_loadcell_Nm,
                               g_pwm_duty_output,
                               g_measured_speed_rpm,
                               g_mechanical_power_w,
                               h_road_load.v_kmh,
                               h_road_load.force_n);
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 (HX711 #1 DOUT) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 (HX711 #1 SCK) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 (HX711 #2 DOUT) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 (HX711 #2 SCK) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
