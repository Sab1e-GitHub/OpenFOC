/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "foc.h"
#include "ina181ax.h"
#include "as5600.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_SHUNT 0.001f
#define ADC_MAX_VALUE 4095
#define PWM_Period 4e-5;

#define DEFALUT_POSITION_PID_KP 100
#define DEFALUT_POSITION_PID_KI 0.01f
#define DEFALUT_POSITION_PID_KD 0.1f
#define DEFALUT_POSITIOM_PID_OUT_MAX 30

#define DEFALUT_SPEED_PID_KP 0.02f
#define DEFALUT_SPEED_PID_KI 0.1f
#define DEFALUT_SPEED_PID_KD 0.0f
#define DEFALUT_SPEED_PID_OUT_MAX 5

#define DEFALUT_IQ_PID_KP 1.2f
#define DEFALUT_IQ_PID_KI 3.5f
#define DEFALUT_IQ_PID_KD 0.0f
#define DEFALUT_IQ_PID_OUT_MAX 6

#define DEFALUT_ID_PID_KP 0.2f
#define DEFALUT_ID_PID_KI 0.2f
#define DEFALUT_ID_PID_KD 0.0f
#define DEFALUT_ID_PID_OUT_MAX 6

#define DEFAULT_IQ_LPF_TS 0.2f
#define DEFAULT_ID_LPF_TS 0.2f
#define DEFAULT_SPEED_LPF_TS 0.02f

#define FOC_MODE_ANGLE_OPENLOOP
//#define FOC_MODE_ANGLE_CLOSEDLOOP
//#define FOC_MODE_VELOCITY_OPENLOOP
//#define FOC_MODE_VELOCITY_CLOSEDLOOP
//#define FOC_MODE_CURRENT_CONTROL

//#define PHASE_AUTO_DETECT

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
ADC_Config_TypeDef adc1;
ADC_Config_TypeDef adc2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

INA181AX_TypeDef ina181ax_m1_ch1, ina181ax_m1_ch2;
AS5600_TypeDef as5600;

FOC_Instance foc;

float angle;
float i_val[2];

uint64_t micros64(void)
{
    static uint32_t last_cycle = 0;
    static uint64_t micros_accum = 0;

    uint32_t current_cycle = DWT->CYCCNT;
    uint32_t cycle_delta = current_cycle - last_cycle;
    last_cycle = current_cycle;

    micros_accum += (uint64_t)cycle_delta / (SystemCoreClock / 1000000UL);
    return micros_accum;
}
void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT 模块
    DWT->CYCCNT = 0;                                // 复位计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启用 CYCCNT
}
/**
 * @brief 设置 PWM 占空比
 * @param htim         定时器句柄指针（例如 &htim1）
 * @param channel      通道（例如 TIM_CHANNEL_1）
 * @param duty_cycle   占空比（范围：0.0f ~ 1.0f）
 */
void pwm_set_duty(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle)
{
    if (htim == NULL || duty_cycle < 0.0f || duty_cycle > 1.0f)
        return;

    // 获取当前计数器周期（Period）
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);

    // 计算新的占空比（Pulse）
    uint32_t pulse = (uint32_t)(duty_cycle * (float)(period + 1));

    // 设置比较值（即占空比）
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void set_pwm(float ch1,float ch2,float ch3){
	pwm_set_duty(&htim2, TIM_CHANNEL_1, ch1);
	pwm_set_duty(&htim2, TIM_CHANNEL_2, ch2);
	pwm_set_duty(&htim2, TIM_CHANNEL_3, ch3);
}
float read_angle(){
	return as5600_read_angle_rad(&as5600);
}
void read_currents(float *i1,float *i2){
	float i2_raw;
	ina181ax_read_current(&ina181ax_m1_ch1, i1);
	ina181ax_read_current(&ina181ax_m1_ch2, &i2_raw);
	*i2 = -i2_raw;
}

// 相序检测函数，不准确，尚不可用
//FOC_Status foc_auto_detect_phase_order(FOC_Instance *user_foc)
//{
//    printf("Phase order auto-detection started...\r\n");
//
//    Motor_Phase phase_orders[6][3] = {
//        {FOC_PHASE_A, FOC_PHASE_B, FOC_PHASE_C},
//        {FOC_PHASE_A, FOC_PHASE_C, FOC_PHASE_B},
//        {FOC_PHASE_B, FOC_PHASE_A, FOC_PHASE_C},
//        {FOC_PHASE_B, FOC_PHASE_C, FOC_PHASE_A},
//        {FOC_PHASE_C, FOC_PHASE_A, FOC_PHASE_B},
//        {FOC_PHASE_C, FOC_PHASE_B, FOC_PHASE_A}};
//    const char *phase_names[6] = {"ABC", "ACB", "BAC", "BCA", "CAB", "CBA"};
//
//    float best_score = -1.0f;
//    int best_index = -1;
//
//    for (int i = 0; i < 6; i++)
//    {
//        printf("Testing order: %s\r\n", phase_names[i]);
//
//        FOC_Instance foc = *user_foc;
//        foc_set_phase(&foc, phase_orders[i][0], phase_orders[i][1], phase_orders[i][2]);
//
//        foc.cfg.mode = FOC_Mode_Velocity_Openloop;
//        foc.cfg.zero_electric_angle = 0.0f;
//        foc.cfg.mode_params.vel_open.uq_openloop = 1.5f;
//        foc.cfg.mode_params.vel_open.target_speed_rad_per_sec = 1.0f;
//
//        foc_calibrate_zero_electric_angle(&foc);
//
//        // 步骤一：检测旋转方向
//        float angle_start = unwrap_angle(foc.cfg.angle_sensor_direction * foc.cfg.callbacks.read_angle(), &foc.state);
//        float angle_last = angle_start;
//        float delta_angle_sum = 0.0f;
//
//        for (int j = 0; j < 300; j++)
//        {
//            foc_run(&foc);
//            foc_delay_ms(foc.cfg.get_micros, 1);
//            float angle_now = unwrap_angle(foc.cfg.angle_sensor_direction * foc.cfg.callbacks.read_angle(), &foc.state);
//            delta_angle_sum += angle_now - angle_last;
//            angle_last = angle_now;
//        }
//
//        bool is_forward = (delta_angle_sum > 0.2f);
//        printf(" -> Net=%.2f rad, Dir=%s\r\n", delta_angle_sum, is_forward ? "FWD" : "REV");
//
//        if (!is_forward)
//        {
//            printf(" -> Skipped: reverse direction.\r\n");
//            continue;
//        }
//
//        // 步骤二：速度闭环电流幅值评分
//        foc.cfg.mode = FOC_Mode_Velocity_Closedloop;
//        foc.cfg.mode_params.vel_closed.target_speed_rad_per_sec = 2.0f;
//        pid_init(&foc.cfg.speed_pid_controller,
//         user_foc->cfg.mode_params.vel_closed.speed_pid.kp,
//         user_foc->cfg.mode_params.vel_closed.speed_pid.ki,
//         user_foc->cfg.mode_params.vel_closed.speed_pid.kd,
//         -user_foc->cfg.mode_params.vel_closed.speed_pid.out_max,
//         user_foc->cfg.mode_params.vel_closed.speed_pid.out_max,
//         foc.cfg.get_micros);
//pid_init(&foc.cfg.iq_pid_controller,
//         user_foc->cfg.mode_params.vel_closed.iq_pid.kp,
//         user_foc->cfg.mode_params.vel_closed.iq_pid.ki,
//         user_foc->cfg.mode_params.vel_closed.iq_pid.kd,
//         -user_foc->cfg.mode_params.vel_closed.iq_pid.out_max,
//         user_foc->cfg.mode_params.vel_closed.iq_pid.out_max,
//         foc.cfg.get_micros);
//pid_init(&foc.cfg.id_pid_controller,
//         user_foc->cfg.mode_params.vel_closed.id_pid.kp,
//         user_foc->cfg.mode_params.vel_closed.id_pid.ki,
//         user_foc->cfg.mode_params.vel_closed.id_pid.kd,
//         -user_foc->cfg.mode_params.vel_closed.id_pid.out_max,
//         user_foc->cfg.mode_params.vel_closed.id_pid.out_max,
//         foc.cfg.get_micros);
//
//        float current_sum = 0.0f;
//        for (int j = 0; j < 200; j++)
//        {
//            foc_run(&foc);
//            foc_delay_ms(foc.cfg.get_micros, 1);
//            float i_mag = fabsf(foc.state.ia) + fabsf(foc.state.ib) + fabsf(foc.state.ic);
//            current_sum += i_mag;
//        }
//
//        float current_avg = current_sum / 200.0f;
//        float score = 1.0f / (current_avg + 0.01f);
//
//        printf(" -> I_avg=%.2f A, Score=%.2f\r\n", current_avg, score);
//
//        if (score > best_score)
//        {
//            best_score = score;
//            best_index = i;
//        }
//    }
//
//    if (best_index >= 0)
//    {
//        foc_set_phase(user_foc,
//                      phase_orders[best_index][0],
//                      phase_orders[best_index][1],
//                      phase_orders[best_index][2]);
//        printf("Best order: %s (Score=%.2f)\r\n", phase_names[best_index], best_score);
//        return FOC_OK;
//    }
//    else
//    {
//        printf("Phase order detection failed!\r\n");
//        return FOC_ERR;
//    }
//}

void main_init(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, GPIO_PIN_SET);

	dwt_init();

	adc_init(&adc1, &hadc1, ADC_CHANNEL_3, ADC_MAX_VALUE,
					ADC_SAMPLETIME_13CYCLES_5);
	adc_init(&adc2, &hadc1, ADC_CHANNEL_4, ADC_MAX_VALUE,
					ADC_SAMPLETIME_13CYCLES_5);

	ina181ax_init(&ina181ax_m1_ch1, R_SHUNT, 3.3f, INA181A2,
	I_SENSOR_CH1_GPIO_Port, I_SENSOR_CH1_Pin, &adc1);
	ina181ax_init(&ina181ax_m1_ch2, R_SHUNT, 3.3f, INA181A2,
	I_SENSOR_CH2_GPIO_Port, I_SENSOR_CH2_Pin, &adc2);

	as5600_init(&as5600, &hi2c1, 0x36, AS5600_CW);

	FOC_InitConfig init_cfg={0};
	init_cfg.angle_sensor_direction = FOC_Direction_CW;
	init_cfg.phase_ch1 = FOC_PHASE_A;
	init_cfg.phase_ch2 = FOC_PHASE_B;
	init_cfg.phase_ch3 = FOC_PHASE_C;

	init_cfg.pole_pairs = 7;
	init_cfg.pwm_period = PWM_Period;
	init_cfg.read_angle = read_angle;
	init_cfg.read_mode = READ_CURRENTS_2_PHASE;
	init_cfg.read_currents_2 = read_currents;
	init_cfg.set_pwm = set_pwm;
	init_cfg.v_dc = 12.0f;

	init_cfg.get_micros = micros64;


#ifdef FOC_MODE_ANGLE_OPENLOOP

	init_cfg.mode = FOC_Mode_Angle_Openloop;
	init_cfg.mode_params.angle_open.uq_openloop = 1.5f;

#elif defined(FOC_MODE_ANGLE_CLOSEDLOOP)

	init_cfg.mode = FOC_Mode_Angle_Closedloop;
	init_cfg.mode_params.angle_closed.target_position_deg = 0.0f;
	init_cfg.mode_params.angle_closed.angle_closedloop_kv = 0.1f;

	init_cfg.mode_params.angle_closed.position_pid.kp = DEFALUT_POSITION_PID_KP;
	init_cfg.mode_params.angle_closed.position_pid.ki = DEFALUT_POSITION_PID_KI;
	init_cfg.mode_params.angle_closed.position_pid.kd = DEFALUT_POSITION_PID_KD;
	init_cfg.mode_params.angle_closed.position_pid.out_max = DEFALUT_POSITIOM_PID_OUT_MAX;

	init_cfg.mode_params.angle_closed.speed_pid.kp = DEFALUT_SPEED_PID_KP;
	init_cfg.mode_params.angle_closed.speed_pid.ki = DEFALUT_SPEED_PID_KI;
	init_cfg.mode_params.angle_closed.speed_pid.kd = DEFALUT_SPEED_PID_KD;
	init_cfg.mode_params.angle_closed.speed_pid.out_max = DEFALUT_SPEED_PID_OUT_MAX;

	init_cfg.mode_params.angle_closed.iq_pid.kp = DEFALUT_IQ_PID_KP;
	init_cfg.mode_params.angle_closed.iq_pid.ki = DEFALUT_IQ_PID_KI;
	init_cfg.mode_params.angle_closed.iq_pid.kd = DEFALUT_IQ_PID_KD;
	init_cfg.mode_params.angle_closed.iq_pid.out_max = DEFALUT_IQ_PID_OUT_MAX;

	init_cfg.mode_params.angle_closed.id_pid.kp = DEFALUT_ID_PID_KP;
	init_cfg.mode_params.angle_closed.id_pid.ki = DEFALUT_ID_PID_KI;
	init_cfg.mode_params.angle_closed.id_pid.kd = DEFALUT_ID_PID_KD;
	init_cfg.mode_params.angle_closed.id_pid.out_max = DEFALUT_ID_PID_OUT_MAX;


	init_cfg.mode_params.angle_closed.lpf_iq_ts = DEFAULT_IQ_LPF_TS;
	init_cfg.mode_params.angle_closed.lpf_id_ts = DEFAULT_ID_LPF_TS;

#elif defined(FOC_MODE_VELOCITY_OPENLOOP)
	init_cfg.mode = FOC_Mode_Velocity_Openloop;
	init_cfg.mode_params.vel_open.uq_openloop = 3.0f;
	init_cfg.mode_params.vel_open.target_speed_rad_per_sec = 50.0f;
#elif defined(FOC_MODE_VELOCITY_CLOSEDLOOP)
	init_cfg.mode = FOC_Mode_Velocity_Closedloop;
	init_cfg.mode_params.vel_closed.target_speed_rad_per_sec = 50.0f;

	init_cfg.mode_params.vel_closed.speed_pid.kp = DEFALUT_SPEED_PID_KP;
	init_cfg.mode_params.vel_closed.speed_pid.ki = DEFALUT_SPEED_PID_KI;
	init_cfg.mode_params.vel_closed.speed_pid.kd = DEFALUT_SPEED_PID_KD;
	init_cfg.mode_params.vel_closed.speed_pid.out_max = DEFALUT_SPEED_PID_OUT_MAX;

	init_cfg.mode_params.vel_closed.iq_pid.kp = DEFALUT_IQ_PID_KP;
	init_cfg.mode_params.vel_closed.iq_pid.ki = DEFALUT_IQ_PID_KI;
	init_cfg.mode_params.vel_closed.iq_pid.kd = DEFALUT_IQ_PID_KD;
	init_cfg.mode_params.vel_closed.iq_pid.out_max = DEFALUT_IQ_PID_OUT_MAX;

	init_cfg.mode_params.vel_closed.id_pid.kp = DEFALUT_ID_PID_KP;
	init_cfg.mode_params.vel_closed.id_pid.ki = DEFALUT_ID_PID_KI;
	init_cfg.mode_params.vel_closed.id_pid.kd = DEFALUT_ID_PID_KD;
	init_cfg.mode_params.vel_closed.id_pid.out_max = DEFALUT_ID_PID_OUT_MAX;

	init_cfg.mode_params.vel_closed.lpf_iq_ts = DEFAULT_IQ_LPF_TS;
	init_cfg.mode_params.vel_closed.lpf_id_ts = DEFAULT_ID_LPF_TS;
#elif defined(FOC_MODE_CURRENT_CONTROL)
	init_cfg.mode = FOC_Mode_Current_Control;
	init_cfg.mode_params.current.target_id = 0.0f;
	init_cfg.mode_params.current.target_iq = 0.2f;

	init_cfg.mode_params.current.iq_pid.kp = DEFALUT_IQ_PID_KP;
	init_cfg.mode_params.current.iq_pid.ki = DEFALUT_IQ_PID_KI;
	init_cfg.mode_params.current.iq_pid.kd = DEFALUT_IQ_PID_KD;
	init_cfg.mode_params.current.iq_pid.out_max = DEFALUT_IQ_PID_OUT_MAX;

	init_cfg.mode_params.current.id_pid.kp = DEFALUT_ID_PID_KP;
	init_cfg.mode_params.current.id_pid.ki = DEFALUT_ID_PID_KI;
	init_cfg.mode_params.current.id_pid.kd = DEFALUT_ID_PID_KD;
	init_cfg.mode_params.current.id_pid.out_max = DEFALUT_ID_PID_OUT_MAX;
#endif
	init_cfg.mode_params.angle_open.lpf_speed_ts = DEFAULT_SPEED_LPF_TS;


	foc_init(&foc, &init_cfg);

#ifdef PHASE_AUTO_DETECT
	foc_auto_detect_phase_order(&foc);
#endif
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	main_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef FOC_MODE_ANGLE_OPENLOOP
		static float target_angle = 0.0f;
		target_angle += 0.5f;  // 每循环增加0.5度（调整此值改变速度）
		if (target_angle >= 360.0f) target_angle = 0.0f;
		foc.cfg.mode_params.angle_open.target_position_deg = target_angle;  // 更新目标角度
		foc_run(&foc);                                // 运行FOC计算
		HAL_Delay(10);                                // 控制更新速率
#elif defined(FOC_MODE_ANGLE_CLOSEDLOOP)
		static uint64_t count = 0;
		count++;
		if(count== 10000){
			count = 0;
			foc.cfg.mode_params.angle_closed.target_position_deg += 60.0f;
			if(foc.cfg.mode_params.angle_closed.target_position_deg >= 360.0f)
				foc.cfg.mode_params.angle_closed.target_position_deg = 0.0f;
		}
		foc_run(&foc);                                // 运行FOC计算
#elif defined(FOC_MODE_VELOCITY_OPENLOOP)
		foc_run(&foc);                                // 运行FOC计算
#elif defined(FOC_MODE_VELOCITY_CLOSEDLOOP)
		foc_run(&foc);                                // 运行FOC计算
#elif defined(FOC_MODE_CURRENT_CONTROL)
		foc_run(&foc);                                // 运行FOC计算
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 1439;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : M1_EN_Pin */
  GPIO_InitStruct.Pin = M1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_EN_GPIO_Port, &GPIO_InitStruct);

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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
