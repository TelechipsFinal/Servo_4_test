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
#define SUSPENSION_MAX_HEIGHT       80     // 100% (최대)
#define SUSPENSION_NEUTRAL          30      // 평소 높이(작을수록 높은 것)

/* PID 제어 파라미터 */
#define UPDATE_RATE_MS              10      // 업데이트 RATE_MS - 고정
#define MAX_CORRECTION              100      // 최대 보정량 (%) - 고정
#define DEADBAND                    0.08f    // 데드밴드 (도)

/* 차량 기하학 파라미터 (실제 차량에 맞게 조정 필요!) */
#define WHEELBASE_MM                223.0f   // 축간 거리 (mm) - 앞뒤 바퀴 간격
#define TRACK_WIDTH_MM              175.0f   // 좌우 바퀴 간격 (mm)
#define SUSPENSION_TRAVEL_MM        23.0f    // 서스펜션 스트로크 (mm) - 25%~80% 범위
#define HEIGHT_TO_PERCENT           2.39f     // 1mm = 2.5% (50% 범위 / 20mm)


/* PID 게인 */
#define LEVELING_GAIN           2.9f    // 수평 유지 강도
#define INTEGRAL_GAIN           0.25f    // I 게인
#define INTEGRAL_MAX            15.0f   // 적분 제한값
#define DERIVATIVE_GAIN         0.02f    // D 게인: 각속도 변화 대응
#define DERIVATIVE_FILTER       0.15f    // D 항 로우패스 필터 (노이즈 감소)

// Kalman Filter 파라미터
//Q : 자이로 적분으로 예측한 각도가 실제와 얼마나 다를 수 있나?
//R : 가속도계로 측정한 각도의 노이즈 레벨
#define KALMAN_Q_ANGLE    0.003f    // 작으면 자이로 중심
#define KALMAN_Q_BIAS     0.00002f  // 드리프트 천천히 제거
#define KALMAN_R_MEASURE  0.1f      // 작으면 가속도계 중심

/* 스무딩 파라미터 */
#define SMOOTH_ALPHA  0.25f  // 0~1, 작을수록 부드러움

/* 가변 댐핑 (충격 감지용) */
#define SHOCK_THRESHOLD         1.8f    // 1.8G 이상의 충격 감지 시 댐핑 모드
#define SHOCK_DECAY             0.95f   // 충격 후 서서히 제어권 복귀
#define SHOCK_CONTROL           0.3f    // 충격 시 제어 강도 (30%)
#define SMOOTH_BASE             0.5f    // 스무딩 기본값
#define SMOOTH_RANGE            0.2f    // 스무딩 가변 범위

/* 서보 인덱스 */
typedef enum {
    SERVO_FRONT_LEFT  = 0,
    SERVO_FRONT_RIGHT = 1,
    SERVO_REAR_LEFT   = 2,
    SERVO_REAR_RIGHT  = 3,
    SERVO_COUNT       = 4
} ServoIndex;

/* Kalman Filter 구조체 */
typedef struct {
    float angle;
    float bias;       // 바이어스 추정
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_measure;
} KalmanFilter;


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

/* PID 제어용 구조체 */
typedef struct {
    float error;
    float prev_error;
    float integral;
    float derivative;
    float filtered_derivative;
    float output;
} PID_Controller;

IMU_Data imu = {0};
Servo servos[SERVO_COUNT];
uint32_t last_update_time = 0;
float roll_offset = 0.0f;
float pitch_offset = 0.0f;

// PID 제어기
PID_Controller pid_roll = {0};
PID_Controller pid_pitch = {0};

float smoothed_targets[4] = {30, 30, 30, 30};

KalmanFilter kalman_roll;
KalmanFilter kalman_pitch;


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
uint16_t Constrain_u16(uint16_t value, uint16_t min, uint16_t max);
float Constrain_f(float value, float min, float max);

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

void Kalman_Init(KalmanFilter *kf) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;  // ⭐ 바이어스 초기값

    // P 행렬 초기화 (2x2 단위 행렬)
    kf->P[0][0] = 1.0f;  // 각도 불확실성
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;  // 바이어스 불확실성

    // 노이즈 파라미터
    kf->Q_angle = KALMAN_Q_ANGLE;
    kf->Q_bias = KALMAN_Q_BIAS;
    kf->R_measure = KALMAN_R_MEASURE;
}

float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt)
{
    // Predict
    float rate = newRate - kf->bias;
    kf->angle += dt * rate;

    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 적응형 R 계산 (추가된 부분)
    // 진동이나 회전이 강할수록 R값을 키워서 가속도계의 오차를 무시합니다.
    float motion = fabsf(newRate);
    float adaptive_R = kf->R_measure * (1.0f + motion);

    // Update
    float y = newAngle - kf->angle;
    float S = kf->P[0][0] + adaptive_R;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];

    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;

    return kf->angle;
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
    servos[SERVO_REAR_RIGHT].channel = TIM_CHANNEL_3;

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
    const int samples = 150;  // 5초 @ 50Hz

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
 * @brief 각도를 서스펜션 높이 변화량(%)으로 변환
 *
 * @param angle_deg 각도 (도)
 * @param distance_mm 해당 축의 거리 (wheelbase 또는 track width)
 * @return 필요한 높이 변화량 (%)
 */
float Angle_To_Height_Change(float angle_deg, float distance_mm) {
    // 각도를 라디안으로 변환
    float angle_rad = angle_deg * M_PI / 180.0f;

    // 필요한 높이 차이 = distance * sin(angle)
    float height_diff_mm = distance_mm * sinf(angle_rad);

    // mm를 %로 변환
    float height_percent = height_diff_mm * HEIGHT_TO_PERCENT;

    return height_percent;
}

void PID_Compute(PID_Controller *pid, float error, float dt) {
    pid->error = error;

    // I: 적분 제어
    if(fabsf(error) > DEADBAND) {
        pid->integral += error * dt;
        pid->integral = Constrain_f(pid->integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    } else {
        pid->integral *= 0.98f;
    }

    // D: 미분 제어
    float raw_derivative = (error - pid->prev_error) / dt;
    pid->filtered_derivative = (DERIVATIVE_FILTER * pid->filtered_derivative) +
                               ((1.0f - DERIVATIVE_FILTER) * raw_derivative);
    pid->derivative = pid->filtered_derivative;
    pid->prev_error = error;

    // PID 출력 = P + I + D
    pid->output = error * LEVELING_GAIN +
                  pid->integral * INTEGRAL_GAIN +
                  pid->derivative * DERIVATIVE_GAIN;
}

/**
 * @brief PID 제어를 사용한 실시간 수평 유지 (기하학적 변환 적용)
 */
void Calculate_Independent_Suspension_Heights_PID(float dt) {
    // 1. 진동 감지 로직 (부드러운 감쇄)
    static float prev_roll = 0.0f, prev_pitch = 0.0f;
    static float p_r_chg = 0.0f, p_p_chg = 0.0f;
    static int osc_count = 0;

    float r_chg = imu.filtered_roll - prev_roll;
    float p_chg = imu.filtered_pitch - prev_pitch;

    // 방향 전환 빈도가 높으면 진동으로 간주
    if((r_chg * p_r_chg < 0) || (p_chg * p_p_chg < 0)) osc_count = 8;
    else if(osc_count > 0) osc_count--;

    prev_roll = imu.filtered_roll; prev_pitch = imu.filtered_pitch;
    p_r_chg = r_chg; p_p_chg = p_chg;

    // 2. 부스트 및 댐핑 결정
    float boost = 1.0f;
    float total_tilt = fabsf(imu.filtered_roll) + fabsf(imu.filtered_pitch);

    // 방지턱 모드: 기울기가 크면 파워 업
    if (total_tilt > 3.0f) boost = 1.6f;

    // 진동 모드: 떨림이 감지되면 파워 다운 (부스트보다 진동 억제 우선)
    float osc_damping = (osc_count > 0) ? 0.6f : 1.0f;

    // 3. 기하학적 높이 변화량 계산
    float r_h_change = Angle_To_Height_Change(imu.filtered_roll, TRACK_WIDTH_MM / 2.0f);
    float p_h_change = Angle_To_Height_Change(imu.filtered_pitch, WHEELBASE_MM / 2.0f);

    // 4. PID 실행
    PID_Compute(&pid_roll, r_h_change, dt);
    PID_Compute(&pid_pitch, p_h_change, dt);

    float r_ctrl = pid_roll.output * boost * osc_damping;
    float p_ctrl = pid_pitch.output * boost * osc_damping;

    // 5. 서보 타겟 할당 (부호 주의: 작을수록 차고 높음)
    // 차체가 앞으로 숙여지면(Pitch +) 앞바퀴를 밀어내야 함(수치 감소)
    float target_f[4];
    target_f[SERVO_FRONT_LEFT]  = (float)SUSPENSION_NEUTRAL + r_ctrl - p_ctrl;
    target_f[SERVO_FRONT_RIGHT] = (float)SUSPENSION_NEUTRAL - r_ctrl - p_ctrl;
    target_f[SERVO_REAR_LEFT]   = (float)SUSPENSION_NEUTRAL + r_ctrl + p_ctrl;
    target_f[SERVO_REAR_RIGHT]  = (float)SUSPENSION_NEUTRAL - r_ctrl + p_ctrl;

    // 6. 가변 스무딩 적용 및 저장
    for(int i = 0; i < SERVO_COUNT; i++) {
        float val = Constrain_f(target_f[i], SUSPENSION_MIN_HEIGHT, SUSPENSION_MAX_HEIGHT);
        // 평소엔 부드럽게, 방지턱에선(boost > 1.0) 더 빠르게 반응
        float alpha = (boost > 1.0f) ? 0.95f : SMOOTH_BASE;
        smoothed_targets[i] = (alpha * val) + ((1.0f - alpha) * smoothed_targets[i]);
        servos[i].target_height = (uint16_t)(smoothed_targets[i] + 0.5f);
    }
}

/**
 * @brief 서보를 목표 높이로 빠르게 추종
 */
void Update_Servos_Gradually(void) {
    for(int i = 0; i < SERVO_COUNT; i++) {
        float gap = (float)servos[i].target_height - servos[i].current_height;

        // 훨씬 빠른 추종 속도
        float step_gain;
        if(fabsf(gap) > 15.0f) {
            step_gain = 0.25f;  // 0.10 → 0.35 (대폭 증가)
        } else if(fabsf(gap) > 5.0f) {
            step_gain = 0.08f;  // 0.08 → 0.25
        } else if(fabsf(gap) > 3.0f) {
            step_gain = 0.03f;  // 0.03 → 0.15
        } else {
            step_gain = 0.08f;  // 미세 조정
        }

        float step = gap * step_gain;

        if (fabsf(gap) > 0.05f) {  // 0.1 → 0.05
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
  UART_SendString("  PID Auto-Leveling System\r\n");
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

  Kalman_Init(&kalman_roll);
  Kalman_Init(&kalman_pitch);

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

      // 100Hz 제어 루프
      if (current_time - last_update_time >= UPDATE_RATE_MS) {
          float dt = (current_time - last_update_time) / 1000.0f;
          last_update_time = current_time;

          // IMU 데이터 읽기
          ICM20948_ReadAccelGyro();
          Calculate_Roll_Pitch();
          imu.filtered_roll = Kalman_Update(&kalman_roll, imu.roll, imu.gyro_x, dt);
          imu.filtered_pitch = Kalman_Update(&kalman_pitch, imu.pitch, imu.gyro_y, dt);

          // PID 제어로 서스펜션 높이 계산 및 적용
          Calculate_Independent_Suspension_Heights_PID(dt);
          Update_Servos_Gradually();

          loop_count++;
      }

    // 500ms마다 데이터 출력
    if (current_time - last_print_time >= 500) {
        last_print_time = current_time;

        UART_SendString("\r\n=== PID Control Status ===\r\n");

        sprintf(buffer, "Roll: %.2f deg, Pitch: %.2f deg\r\n", imu.filtered_roll, imu.filtered_pitch);
        UART_SendString(buffer);

        sprintf(buffer, "PID Pitch - P:%.2f I:%.2f D:%.2f Out:%.2f\r\n",
                pid_pitch.error * LEVELING_GAIN,
                pid_pitch.integral * INTEGRAL_GAIN,
                pid_pitch.derivative * DERIVATIVE_GAIN,
                pid_pitch.output);
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
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
