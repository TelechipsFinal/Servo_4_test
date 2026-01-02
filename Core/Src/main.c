/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ICM-20948 + MG996R Fast Servo Control
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ICM-20948 Bank 레지스터 */
#define ICM20948_REG_BANK_SEL       0x7F

/* Bank 0 레지스터 */
#define ICM20948_WHO_AM_I           0x00
#define ICM20948_USER_CTRL          0x03
#define ICM20948_PWR_MGMT_1         0x06
#define ICM20948_PWR_MGMT_2         0x07
#define ICM20948_ACCEL_XOUT_H       0x2D
#define ICM20948_GYRO_XOUT_H        0x33

/* Bank 2 레지스터 */
#define ICM20948_GYRO_CONFIG_1      0x01
#define ICM20948_ACCEL_CONFIG       0x14

/* ICM-20948 설정값 */
#define ICM20948_DEVICE_ID          0xEA

/* 핀 정의 */
#define ICM20948_CS_PIN             GPIO_PIN_4
#define ICM20948_CS_PORT            GPIOA

/* 서보모터 PWM 설정 */
#define SERVO_MIN_PULSE    500     // 0.5ms (0도)
#define SERVO_MAX_PULSE    2500    // 2.5ms (180도)
#define SERVO_CENTER_PULSE 1500    // 1.5ms (90도)

/* 차량 서스펜션 설정 */
#define SUSPENSION_MIN_HEIGHT       25       // 0% (최소)
#define SUSPENSION_MAX_HEIGHT       75     // 100% (최대)
#define SUSPENSION_NEUTRAL          30      // 평소 높이(작을수록 높은 것)

/* 제어 파라미터 */
#define UPDATE_RATE_MS              10      // 100Hz 업데이트
#define ROLL_GAIN                   2.0f    // Roll 보상 게인
#define PITCH_GAIN                  2.0f    // Pitch 보상 게인
#define MAX_CORRECTION              55      // 최대 보정량 (%)
#define COMPLEMENTARY_ALPHA         0.98f   // 상보 필터 계수
#define DEADBAND                    0.1f    // 데드밴드 (도)

#define HEIGHT_CHANGE_SPEED     0.5f       // 한 번에 변경할 높이 (%, 클수록 빠름)
#define LEVELING_GAIN           5.0f    // 수평 유지 강도 (높을수록 강함)
#define MIN_ANGLE_CHANGE        0.8f    // 최소 감지 각도 (도)

/* 서보 인덱스 */
typedef enum {
    SERVO_FRONT_LEFT  = 0,
    SERVO_FRONT_RIGHT = 1,
    SERVO_REAR_LEFT   = 2,
    SERVO_REAR_RIGHT  = 3,
    SERVO_COUNT       = 4
} ServoIndex;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* 센서 데이터 구조체 */
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float roll, pitch;
    float filtered_roll, filtered_pitch;
} IMU_Data;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_height;    // 0~100%
    uint16_t target_height;     // 0~100%
} Servo;

IMU_Data imu = {0};
Servo servos[SERVO_COUNT];
uint32_t last_update_time = 0;
float roll_offset = 0.0f;
float pitch_offset = 0.0f;
float prev_target_heights[4] = {30, 30, 30, 30};  // ← 추가

// ✅ 추가: 적분 제어
float integral_roll = 0.0f;
float integral_pitch = 0.0f;
#define INTEGRAL_GAIN   0.4f
#define INTEGRAL_MAX    30.0f

float smoothed_targets[4] = {30, 30, 30, 30};  // ← 추가
#define SMOOTH_ALPHA  0.15f  // 0~1, 작을수록 부드러움

// CAN 제어 변수 추가
static volatile uint8_t g_suspension_enabled = 0;  // 0: OFF, 1: ON
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void UART_SendString(char* str);
void ICM20948_CS_Low(void);
void ICM20948_CS_High(void);
void ICM20948_SelectBank(uint8_t bank);
void ICM20948_WriteReg(uint8_t reg, uint8_t data);
uint8_t ICM20948_ReadReg(uint8_t reg);
void ICM20948_ReadMultipleReg(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t ICM20948_Init(void);
void ICM20948_ReadAccelGyro(void);
void Calculate_Roll_Pitch(void);
void Apply_Complementary_Filter(float dt);
void Suspension_Init(void);
void Servo_SetHeight(ServoIndex idx, uint16_t height_percent);
void Servo_SetPulse(ServoIndex idx, uint16_t pulse);
uint16_t Constrain_u16(uint16_t value, uint16_t min, uint16_t max);
float Constrain_f(float value, float min, float max);
void Calculate_Suspension_Heights_Independent(uint32_t current_time);
void Update_Z_Accel_History(ServoIndex idx, float z_accel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_SendString(char* str) {
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void ICM20948_CS_Low(void) {
    HAL_GPIO_WritePin(ICM20948_CS_PORT, ICM20948_CS_PIN, GPIO_PIN_RESET);
}

void ICM20948_CS_High(void) {
    HAL_GPIO_WritePin(ICM20948_CS_PORT, ICM20948_CS_PIN, GPIO_PIN_SET);
}

void ICM20948_SelectBank(uint8_t bank) {
    uint8_t tx[2] = {ICM20948_REG_BANK_SEL & 0x7F, bank << 4};
    ICM20948_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    ICM20948_CS_High();
}

void ICM20948_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};
    ICM20948_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    ICM20948_CS_High();
}

uint8_t ICM20948_ReadReg(uint8_t reg) {
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    ICM20948_CS_Low();
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
    HAL_SPI_Receive(&hspi1, &rx, 1, 100);
    ICM20948_CS_High();
    return rx;
}

void ICM20948_ReadMultipleReg(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t tx = reg | 0x80;
    ICM20948_CS_Low();
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
    HAL_SPI_Receive(&hspi1, data, len, 100);
    ICM20948_CS_High();
}

uint8_t ICM20948_Init(void) {
    uint8_t who_am_i;
    char buffer[100];

    HAL_Delay(100);
    ICM20948_SelectBank(0);
    HAL_Delay(10);

    who_am_i = ICM20948_ReadReg(ICM20948_WHO_AM_I);
    sprintf(buffer, "WHO_AM_I: 0x%02X\r\n", who_am_i);
    UART_SendString(buffer);

    if (who_am_i != ICM20948_DEVICE_ID) {
        UART_SendString("Error: Wrong device!\r\n");
        return 1;
    }

    ICM20948_SelectBank(0);
    ICM20948_WriteReg(ICM20948_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    ICM20948_WriteReg(ICM20948_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    ICM20948_WriteReg(ICM20948_PWR_MGMT_2, 0x00);
    HAL_Delay(10);
    ICM20948_WriteReg(ICM20948_USER_CTRL, 0x10);
    HAL_Delay(10);

    ICM20948_SelectBank(2);
    ICM20948_WriteReg(ICM20948_GYRO_CONFIG_1, 0x01);
    HAL_Delay(10);
    ICM20948_WriteReg(ICM20948_ACCEL_CONFIG, 0x01);
    HAL_Delay(10);

    ICM20948_SelectBank(0);
    HAL_Delay(10);

    return 0;
}

void ICM20948_ReadAccelGyro(void) {
    uint8_t data[6];
    int16_t raw[6];

    ICM20948_ReadMultipleReg(ICM20948_ACCEL_XOUT_H, data, 6);
    raw[0] = (int16_t)((data[0] << 8) | data[1]);
    raw[1] = (int16_t)((data[2] << 8) | data[3]);
    raw[2] = (int16_t)((data[4] << 8) | data[5]);

    ICM20948_ReadMultipleReg(ICM20948_GYRO_XOUT_H, data, 6);
    raw[3] = (int16_t)((data[0] << 8) | data[1]);
    raw[4] = (int16_t)((data[2] << 8) | data[3]);
    raw[5] = (int16_t)((data[4] << 8) | data[5]);

    imu.accel_x = raw[0] / 16384.0f;
    imu.accel_y = raw[1] / 16384.0f;
    imu.accel_z = raw[2] / 16384.0f;

    imu.gyro_x = raw[3] / 131.0f;
    imu.gyro_y = raw[4] / 131.0f;
    imu.gyro_z = raw[5] / 131.0f;
}

void Calculate_Roll_Pitch(void) {
	//데이터 계산
	float raw_roll = atan2f(imu.accel_y, imu.accel_z) * 180.0f / M_PI;
	float raw_pitch = atan2f(-imu.accel_x, sqrtf(imu.accel_y * imu.accel_y +
	                  imu.accel_z * imu.accel_z)) * 180.0f / M_PI;

	// 캘리브레이션 오프셋 적용 (빼주어야 함)
	imu.roll = raw_roll - roll_offset;
	imu.pitch = raw_pitch - pitch_offset;
}

void Apply_Complementary_Filter(float dt) {
    float gyro_roll_delta = imu.gyro_x * dt;
    float gyro_pitch_delta = imu.gyro_y * dt;

    imu.filtered_roll = COMPLEMENTARY_ALPHA * (imu.filtered_roll + gyro_roll_delta) +
                        (1.0f - COMPLEMENTARY_ALPHA) * imu.roll;
    imu.filtered_pitch = COMPLEMENTARY_ALPHA * (imu.filtered_pitch + gyro_pitch_delta) +
                         (1.0f - COMPLEMENTARY_ALPHA) * imu.pitch;
}

void Suspension_Init(void) {
    // 서보 TIM 채널 설정
    servos[SERVO_FRONT_LEFT].htim = &htim2;
    servos[SERVO_FRONT_LEFT].channel = TIM_CHANNEL_1;

    servos[SERVO_FRONT_RIGHT].htim = &htim3;
    servos[SERVO_FRONT_RIGHT].channel = TIM_CHANNEL_1;

    servos[SERVO_REAR_LEFT].htim = &htim4;
    servos[SERVO_REAR_LEFT].channel = TIM_CHANNEL_1;

    servos[SERVO_REAR_RIGHT].htim = &htim4;
    servos[SERVO_REAR_RIGHT].channel = TIM_CHANNEL_3;  // CH2로 변경!

    // PWM 시작
    for(int i = 0; i < SERVO_COUNT; i++) {
        HAL_TIM_PWM_Start(servos[i].htim, servos[i].channel);
        servos[i].current_height = SUSPENSION_NEUTRAL;
        servos[i].target_height = SUSPENSION_NEUTRAL;
        Servo_SetHeight(i, SUSPENSION_NEUTRAL);
    }

    HAL_Delay(1000);
    UART_SendString("Suspension initialized to neutral\r\n");
}

void Servo_SetPulse(ServoIndex idx, uint16_t pulse) {
    __HAL_TIM_SET_COMPARE(servos[idx].htim, servos[idx].channel, pulse);
}

void Servo_SetHeight(ServoIndex idx, uint16_t height_percent) {
    height_percent = Constrain_u16(height_percent, SUSPENSION_MIN_HEIGHT, SUSPENSION_MAX_HEIGHT);

    uint16_t pulse = SERVO_MIN_PULSE +
                     (height_percent * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 100;

    Servo_SetPulse(idx, pulse);
    servos[idx].current_height = height_percent;
}

uint16_t Constrain_u16(uint16_t value, uint16_t min, uint16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float Constrain_f(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
/**
 * @brief IMU 캘리브레이션 - 초기 자세를 기준점으로 설정
 *
 * 평평한 곳에서 5초간 측정하여 평균값을 오프셋으로 저장
 */

void IMU_Calibrate(void) {
    char buffer[100];
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    const int samples = 250;  // 5초 @ 50Hz

    UART_SendString("\r\n=== IMU Calibration Start ===\r\n");
    UART_SendString("Keep vehicle on flat surface for 5 seconds...\r\n");

    for(int i = 0; i < samples; i++) {
        ICM20948_ReadAccelGyro();
        Calculate_Roll_Pitch();

        roll_sum += imu.roll;
        pitch_sum += imu.pitch;

        HAL_Delay(20);  // 50Hz

        if(i % 50 == 0) {
            UART_SendString(".");
        }
    }

    roll_offset = roll_sum / samples;
    pitch_offset = pitch_sum / samples;

    sprintf(buffer, "\r\nCalibration complete!\r\n");
    UART_SendString(buffer);
    sprintf(buffer, "Roll offset: %.2f deg, Pitch offset: %.2f deg\r\n\r\n",
            roll_offset, pitch_offset);
    UART_SendString(buffer);
}

/**
 * @brief 실시간 수평 유지 - 각 바퀴 독립 제어
 */
void Calculate_Independent_Suspension_Heights(void) {
    // 1. 입력 필터 (반응성을 위해 가중치 조정)
    static float lpf_roll = 0, lpf_pitch = 0;
    lpf_roll = (lpf_roll * 0.6f) + (imu.filtered_roll * 0.4f);
    lpf_pitch = (lpf_pitch * 0.6f) + (imu.filtered_pitch * 0.4f);

    // 2. 적분 제어 (낮은 각도에서 끝까지 밀어주는 동력)
    // 미세한 각도라도 데드밴드(0.2도)만 넘으면 즉시 적분 시작
    if(fabsf(lpf_roll) > DEADBAND) {
       integral_roll += lpf_roll * 0.05f;
    } else {
       // 데드밴드 이내면 아주 빠르게 0으로 수렴시켜서 "슬금슬금" 움직이는 현상 방지
       integral_roll *= 0.5f;
       if(fabsf(integral_roll) < 0.01f) integral_roll = 0;
    }

    if(fabsf(lpf_pitch) > DEADBAND) {
        integral_pitch += lpf_pitch * 0.05f;
    } else {
        integral_pitch *= 0.5f;
        if(fabsf(integral_pitch) < 0.01f) integral_pitch = 0;
    }

    integral_roll = Constrain_f(integral_roll, -INTEGRAL_MAX, INTEGRAL_MAX);
    integral_pitch = Constrain_f(integral_pitch, -INTEGRAL_MAX, INTEGRAL_MAX);

    // 3. 비선형 제어량 계산 (P를 강화하고 I를 합산)
    // 각도가 낮을 때도 LEVELING_GAIN이 높아서 즉각 반응합니다.
    float r_ctrl = (lpf_roll * LEVELING_GAIN) + (integral_roll * INTEGRAL_GAIN);
    float p_ctrl = (lpf_pitch * LEVELING_GAIN) + (integral_pitch * INTEGRAL_GAIN);

    float target_f[4];
    // 부호 주의: 센서 방향에 맞춰 수평을 잡도록 네 바퀴 연산
    target_f[SERVO_FRONT_LEFT]  = (float)SUSPENSION_NEUTRAL + r_ctrl - p_ctrl;
    target_f[SERVO_FRONT_RIGHT] = (float)SUSPENSION_NEUTRAL - r_ctrl - p_ctrl;
    target_f[SERVO_REAR_LEFT]   = (float)SUSPENSION_NEUTRAL + r_ctrl + p_ctrl;
    target_f[SERVO_REAR_RIGHT]  = (float)SUSPENSION_NEUTRAL - r_ctrl + p_ctrl;

    // 4. 제한 및 필터링
    for(int i = 0; i < SERVO_COUNT; i++) {
        // 중립 기준 최대 이동폭 제한 확인
        float diff = target_f[i] - (float)SUSPENSION_NEUTRAL;
        diff = Constrain_f(diff, -(float)MAX_CORRECTION, (float)MAX_CORRECTION);

        float final_val = (float)SUSPENSION_NEUTRAL + diff;
        final_val = Constrain_f(final_val, (float)SUSPENSION_MIN_HEIGHT, (float)SUSPENSION_MAX_HEIGHT);

        // 지수 이동 평균으로 부드럽게 목표값 전달
        smoothed_targets[i] = (SMOOTH_ALPHA * final_val) + ((1.0f - SMOOTH_ALPHA) * smoothed_targets[i]);
        servos[i].target_height = (uint16_t)(smoothed_targets[i] + 0.5f);
    }
}

/**
 * @brief 서보를 목표 높이로 점진적으로 이동
 */
void Update_Servos_Gradually(void) {
    for(int i = 0; i < SERVO_COUNT; i++) {
        float gap = (float)servos[i].target_height - servos[i].current_height;

        // 추종 계수를 0.08에서 0.12로 상향 (반응 지연 감소)
        float step = gap * 0.12f;

        if (fabsf(gap) > 0.05f) {
            servos[i].current_height += step;
        } else {
            servos[i].current_height = (float)servos[i].target_height;
        }

        Servo_SetHeight(i, (uint16_t)(servos[i].current_height + 0.5f));
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
	char buffer[256];
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
    UART_SendString("\r\n\r\n");
    UART_SendString("=====================================\r\n");
    UART_SendString("  ICM-20948 + 4x Servo Active Suspension\r\n");
    UART_SendString("  Auto-Leveling System\r\n");
    UART_SendString("=====================================\r\n\r\n");

    HAL_Delay(100);

    // ICM-20948 초기화
    if (ICM20948_Init() == 0) {
        UART_SendString("ICM-20948 Init Success!\r\n");
    } else {
        UART_SendString("ICM-20948 Init Failed!\r\n");
        while(1);
    }

    // 서스펜션 초기화
    UART_SendString("Initializing Suspension System...\r\n");
    Suspension_Init();
    UART_SendString("System Ready!\r\n\r\n");

    IMU_Calibrate();

    HAL_Delay(500);

    ICM20948_SelectBank(0);
    last_update_time = HAL_GetTick();

    uint32_t loop_count = 0;
    uint32_t last_print_time = HAL_GetTick();

    UART_SendString("Starting active suspension control...\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
  {
    uint32_t current_time = HAL_GetTick();

    // 50Hz 제어 루프
    if (current_time - last_update_time >= UPDATE_RATE_MS) {
        float dt = (current_time - last_update_time) / 1000.0f;
        last_update_time = current_time;

        // IMU 데이터 읽기
        ICM20948_ReadAccelGyro();
        Calculate_Roll_Pitch();
        Apply_Complementary_Filter(dt);

        // 서스펜션 높이 계산 및 적용
        Calculate_Independent_Suspension_Heights();
        Update_Servos_Gradually();

        loop_count++;
    }

    // 500ms마다 데이터 출력
    if (current_time - last_print_time >= 500) {
        last_print_time = current_time;

        UART_SendString("\r\n=== Independent Wheel Control ===\r\n");

        sprintf(buffer, "Roll: %.2f deg, Pitch: %.2f deg\r\n", imu.filtered_roll, imu.filtered_pitch);
        UART_SendString(buffer);

        sprintf(buffer, "Wheel Heights (%%): FL=%.1f FR=%.1f RL=%.1f RR=%.1f\r\n",
                servos[SERVO_FRONT_LEFT].current_height,
                servos[SERVO_FRONT_RIGHT].current_height,
                servos[SERVO_REAR_LEFT].current_height,
                servos[SERVO_REAR_RIGHT].current_height);
        UART_SendString(buffer);


        sprintf(buffer, "Loop Rate: %lu Hz\r\n", loop_count * 2);
        UART_SendString(buffer);

                loop_count = 0;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
