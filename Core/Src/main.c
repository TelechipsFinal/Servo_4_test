/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ICM-20948 IMU 센서와 MG996R 서보모터를 사용한
  *                   4륜 독립 액티브 서스펜션 제어 시스템
  * @description    : PID 제어와 칼만 필터를 활용하여 차량의 롤/피치를
  *                   실시간으로 보정하고 수평 유지
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>   // sprintf 등 문자열 처리 함수
#include <string.h>  // strlen 등 문자열 길이 함수
#include <math.h>    // 삼각함수, 제곱근 등 수학 함수
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ========================================================================
   ICM-20948 IMU 센서 레지스터 주소 정의
   ======================================================================== */

// Bank 선택 레지스터 (ICM-20948은 여러 Bank로 레지스터가 나뉘어 있음)
#define ICM20948_REG_BANK_SEL      0x7F

/* Bank 0 레지스터들 (기본 동작 관련) */
#define ICM20948_WHO_AM_I          0x00  // 디바이스 ID 확인 레지스터
#define ICM20948_USER_CTRL         0x03  // 사용자 제어 레지스터
#define ICM20948_PWR_MGMT_1        0x06  // 전원 관리 1 레지스터
#define ICM20948_PWR_MGMT_2        0x07  // 전원 관리 2 레지스터
#define ICM20948_ACCEL_XOUT_H      0x2D  // 가속도계 X축 High Byte 시작 주소
#define ICM20948_GYRO_XOUT_H       0x33  // 자이로스코프 X축 High Byte 시작 주소

/* Bank 2 레지스터들 (센서 설정 관련) */
#define ICM20948_GYRO_CONFIG_1     0x01  // 자이로 설정 레지스터
#define ICM20948_ACCEL_CONFIG      0x14  // 가속도계 설정 레지스터

/* ICM-20948 기본 설정값 */
#define ICM20948_DEVICE_ID         0xEA  // WHO_AM_I 응답값 (정상 연결 확인)

/* SPI 통신용 핀 정의 */
#define ICM20948_CS_PIN            GPIO_PIN_4   // Chip Select 핀 번호
#define ICM20948_CS_PORT           GPIOA        // Chip Select 포트

/* ========================================================================
   서보모터 PWM 제어 설정
   ======================================================================== */

// MG996R 서보모터의 PWM 펄스 폭 범위 (마이크로초 단위)
#define SERVO_MIN_PULSE            500   // 0.5ms → 0도 위치
#define SERVO_MAX_PULSE            2500  // 2.5ms → 180도 위치
#define SERVO_CENTER_PULSE         1500  // 1.5ms → 90도 중립 위치

/* ========================================================================
   서스펜션 높이 설정 (% 단위)
   ======================================================================== */

#define SUSPENSION_MIN_HEIGHT      25    // 최소 높이 (25% = 가장 낮음)
#define SUSPENSION_MAX_HEIGHT      80    // 최대 높이 (80% = 가장 높음)
#define SUSPENSION_NEUTRAL         30    // 평상시 높이 (30% = 기본 주행 높이)
                                         // 주의: 숫자가 작을수록 서보가 더 올라감

/* ========================================================================
   PID 제어 파라미터
   ======================================================================== */

#define UPDATE_RATE_MS             10    // 제어 루프 주기 (10ms = 100Hz)
#define MAX_CORRECTION             100   // 최대 보정량 (%)
#define DEADBAND                   0.08f // 데드밴드 각도 (도) - 이 이하는 무시

/* ========================================================================
   차량 기하학적 치수 (실제 차량에 맞게 조정 필요!)
   ======================================================================== */

#define WHEELBASE_MM               223.0f  // 축간 거리 (mm) - 앞뒤 바퀴 간격
#define TRACK_WIDTH_MM             175.0f  // 윤거 (mm) - 좌우 바퀴 간격
#define SUSPENSION_TRAVEL_MM       23.0f   // 서스펜션 스트로크 (mm)
#define HEIGHT_TO_PERCENT          2.39f   // 높이 변환 계수 (1mm = 2.39%)

/* ========================================================================
   PID 제어 게인 값
   ======================================================================== */

#define LEVELING_GAIN              2.9f    // P 게인: 기울기에 대한 즉각 반응 강도
#define INTEGRAL_GAIN              0.25f   // I 게인: 누적 오차 제거 강도
#define INTEGRAL_MAX               15.0f   // I 항 최대값 (와인드업 방지)
#define DERIVATIVE_GAIN            0.02f   // D 게인: 변화율 대응 강도
#define DERIVATIVE_FILTER          0.15f   // D 항 필터 계수 (노이즈 감소)

/* ========================================================================
   칼만 필터 파라미터
   ======================================================================== */

// Q: 프로세스 노이즈 (자이로 적분 예측의 불확실성)
#define KALMAN_Q_ANGLE             0.003f    // 각도 추정 불확실성
#define KALMAN_Q_BIAS              0.00002f  // 바이어스 추정 불확실성

// R: 측정 노이즈 (가속도계 측정의 신뢰도)
#define KALMAN_R_MEASURE           0.1f      // 가속도계 노이즈 레벨

/* ========================================================================
   스무딩 및 충격 감지 파라미터
   ======================================================================== */

#define SMOOTH_ALPHA               0.25f    // 스무딩 계수 (0~1, 작을수록 부드러움)
#define SHOCK_THRESHOLD            1.8f     // 충격 감지 임계값 (G)
#define SHOCK_DECAY                0.95f    // 충격 후 복귀 속도
#define SHOCK_CONTROL              0.3f     // 충격 시 제어 강도 (30%)
#define SMOOTH_BASE                0.5f     // 기본 스무딩 값
#define SMOOTH_RANGE               0.2f     // 스무딩 가변 범위

/* ========================================================================
   서보모터 인덱스 열거형
   ======================================================================== */

typedef enum {
    SERVO_FRONT_LEFT = 0,   // 좌측 전륜
    SERVO_FRONT_RIGHT = 1,  // 우측 전륜
    SERVO_REAR_LEFT = 2,    // 좌측 후륜
    SERVO_REAR_RIGHT = 3,   // 우측 후륜
    SERVO_COUNT = 4         // 총 서보 개수
} ServoIndex;

/* ========================================================================
   칼만 필터 구조체 정의
   ======================================================================== */

typedef struct {
    float angle;        // 현재 추정 각도
    float bias;         // 자이로 바이어스 추정값 (드리프트 보정)
    float P[2][2];      // 오차 공분산 행렬 (2x2)
    float Q_angle;      // 프로세스 노이즈 (각도)
    float Q_bias;       // 프로세스 노이즈 (바이어스)
    float R_measure;    // 측정 노이즈
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

/* ========================================================================
   IMU 센서 데이터 구조체
   ======================================================================== */

typedef struct {
    // 가속도계 원시 데이터 (G 단위)
    float accel_x, accel_y, accel_z;

    // 자이로스코프 원시 데이터 (deg/s 단위)
    float gyro_x, gyro_y, gyro_z;

    // 계산된 롤/피치 각도 (도 단위)
    float roll, pitch;

    // 칼만 필터 적용 후 각도 (도 단위)
    float filtered_roll, filtered_pitch;
} IMU_Data;

/* ========================================================================
   서보모터 제어 구조체
   ======================================================================== */

typedef struct {
    TIM_HandleTypeDef *htim;    // PWM 타이머 핸들 포인터
    uint32_t channel;            // PWM 채널 번호
    float current_height;        // 현재 높이 (0~100%)
    uint16_t target_height;      // 목표 높이 (0~100%)
} Servo;

/* ========================================================================
   PID 제어기 구조체
   ======================================================================== */

typedef struct {
    float error;                 // 현재 오차
    float prev_error;            // 이전 오차 (D 항 계산용)
    float integral;              // 적분 누적값 (I 항)
    float derivative;            // 미분값 (D 항)
    float filtered_derivative;   // 필터링된 미분값 (노이즈 제거)
    float output;                // PID 출력값
} PID_Controller;

/* ========================================================================
   전역 변수 선언
   ======================================================================== */

IMU_Data imu = {0};                      // IMU 센서 데이터 저장 구조체
Servo servos[SERVO_COUNT];               // 4개 서보모터 배열
uint32_t last_update_time = 0;           // 마지막 업데이트 시간 (ms)
float roll_offset = 0.0f;                // 롤 캘리브레이션 오프셋
float pitch_offset = 0.0f;               // 피치 캘리브레이션 오프셋

// PID 제어기 인스턴스
PID_Controller pid_roll = {0};          // 롤 제어용 PID
PID_Controller pid_pitch = {0};         // 피치 제어용 PID

// 스무딩된 목표 높이값 배열
float smoothed_targets[4] = {30, 30, 30, 30};

// 칼만 필터 인스턴스
KalmanFilter kalman_roll;                // 롤 각도 필터
KalmanFilter kalman_pitch;               // 피치 각도 필터

// CAN 통신으로 제어되는 서스펜션 ON/OFF 플래그
volatile uint8_t g_suspension_enabled = 1;  // 0=OFF, 1=ON

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

// 값 제한 함수 프로토타입
uint16_t Constrain_u16(uint16_t value, uint16_t min, uint16_t max);
float Constrain_f(float value, float min, float max);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========================================================================
   UART 문자열 전송 함수
   ======================================================================== */

/**
 * @brief UART를 통해 문자열 전송 (디버깅용)
 * @param str 전송할 문자열 포인터
 */
void UART_SendString(char* str) {
    // HAL_UART_Transmit: UART로 데이터 전송
    // (핸들, 데이터, 길이, 타임아웃)
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/* ========================================================================
   ICM-20948 SPI 통신 함수들
   ======================================================================== */

/**
 * @brief SPI Chip Select 핀을 LOW로 설정 (통신 시작)
 */
void ICM20948_CS_Low(void) {
    HAL_GPIO_WritePin(ICM20948_CS_PORT, ICM20948_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief SPI Chip Select 핀을 HIGH로 설정 (통신 종료)
 */
void ICM20948_CS_High(void) {
    HAL_GPIO_WritePin(ICM20948_CS_PORT, ICM20948_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief ICM-20948의 레지스터 Bank 선택
 * @param bank Bank 번호 (0~3)
 * @note ICM-20948은 레지스터가 4개 Bank로 분리되어 있어
 *       접근 전 해당 Bank를 선택해야 함
 */
void ICM20948_SelectBank(uint8_t bank) {
    // Bank 선택 레지스터에 쓰기 (상위 4비트에 Bank 번호 저장)
    uint8_t tx[2] = {ICM20948_REG_BANK_SEL & 0x7F, bank << 4};

    ICM20948_CS_Low();                              // CS 활성화
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);          // SPI 전송
    ICM20948_CS_High();                             // CS 비활성화
}

/**
 * @brief ICM-20948 레지스터에 1바이트 쓰기
 * @param reg 레지스터 주소
 * @param data 쓸 데이터
 */
void ICM20948_WriteReg(uint8_t reg, uint8_t data) {
    // SPI 쓰기: MSB를 0으로 설정 (& 0x7F)
    uint8_t tx[2] = {reg & 0x7F, data};

    ICM20948_CS_Low();                              // CS 활성화
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);          // 레지스터 주소 + 데이터 전송
    ICM20948_CS_High();                             // CS 비활성화
}

/**
 * @brief ICM-20948 레지스터에서 1바이트 읽기
 * @param reg 레지스터 주소
 * @return 읽은 데이터
 */
uint8_t ICM20948_ReadReg(uint8_t reg) {
    // SPI 읽기: MSB를 1로 설정 (| 0x80)
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;

    ICM20948_CS_Low();                              // CS 활성화
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);         // 레지스터 주소 전송
    HAL_SPI_Receive(&hspi1, &rx, 1, 100);          // 데이터 수신
    ICM20948_CS_High();                             // CS 비활성화

    return rx;
}

/**
 * @brief ICM-20948 레지스터에서 여러 바이트 읽기
 * @param reg 시작 레지스터 주소
 * @param data 데이터를 저장할 버퍼 포인터
 * @param len 읽을 바이트 수
 */
void ICM20948_ReadMultipleReg(uint8_t reg, uint8_t *data, uint8_t len) {
    // SPI 읽기: MSB를 1로 설정 (| 0x80)
    uint8_t tx = reg | 0x80;

    ICM20948_CS_Low();                              // CS 활성화
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);         // 레지스터 주소 전송
    HAL_SPI_Receive(&hspi1, data, len, 100);       // 연속 데이터 수신
    ICM20948_CS_High();                             // CS 비활성화
}

/* ========================================================================
   ICM-20948 초기화 함수
   ======================================================================== */

/**
 * @brief ICM-20948 센서 초기화
 * @return 0: 성공, 1: 실패
 */
uint8_t ICM20948_Init(void) {
    uint8_t who_am_i;
    char buffer[100];

    // 센서 안정화 대기
    HAL_Delay(100);

    // Bank 0으로 전환 (기본 레지스터 접근)
    ICM20948_SelectBank(0);
    HAL_Delay(10);

    // WHO_AM_I 레지스터 읽어서 디바이스 확인
    who_am_i = ICM20948_ReadReg(ICM20948_WHO_AM_I);
    sprintf(buffer, "WHO_AM_I: 0x%02X\r\n", who_am_i);
    UART_SendString(buffer);

    // 디바이스 ID 검증
    if (who_am_i != ICM20948_DEVICE_ID) {
        UART_SendString("Error: Wrong device!\r\n");
        return 1;  // 초기화 실패
    }

    // === 센서 리셋 ===
    ICM20948_SelectBank(0);
    // PWR_MGMT_1에 0x80 쓰기 → 소프트웨어 리셋
    ICM20948_WriteReg(ICM20948_PWR_MGMT_1, 0x80);
    HAL_Delay(100);  // 리셋 완료 대기

    // === 전원 관리 설정 ===
    // PWR_MGMT_1: 0x01 → Auto select best clock source
    ICM20948_WriteReg(ICM20948_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    // PWR_MGMT_2: 0x00 → 가속도계와 자이로 모두 활성화
    ICM20948_WriteReg(ICM20948_PWR_MGMT_2, 0x00);
    HAL_Delay(10);

    // USER_CTRL: 0x10 → I2C 마스터 비활성화 (SPI만 사용)
    ICM20948_WriteReg(ICM20948_USER_CTRL, 0x10);
    HAL_Delay(10);

    // === Bank 2로 전환하여 센서 범위 설정 ===
    ICM20948_SelectBank(2);

    // GYRO_CONFIG_1: 0x01 → ±500 dps, DLPF 활성화
    ICM20948_WriteReg(ICM20948_GYRO_CONFIG_1, 0x01);
    HAL_Delay(10);

    // ACCEL_CONFIG: 0x01 → ±4g, DLPF 활성화
    ICM20948_WriteReg(ICM20948_ACCEL_CONFIG, 0x01);
    HAL_Delay(10);

    // Bank 0으로 복귀
    ICM20948_SelectBank(0);
    HAL_Delay(10);

    return 0;  // 초기화 성공
}

/* ========================================================================
   IMU 데이터 읽기 함수
   ======================================================================== */

/**
 * @brief 가속도계와 자이로스코프 데이터 읽기
 * @note 각 축당 2바이트(High + Low)씩 읽어서 16비트 정수로 변환
 */
void ICM20948_ReadAccelGyro(void) {
    uint8_t data[6];    // 데이터 버퍼 (6바이트 = 3축 * 2바이트)
    int16_t raw[6];     // 원시 센서값 저장 배열

    // === 가속도계 데이터 읽기 (X, Y, Z 순서) ===
    ICM20948_ReadMultipleReg(ICM20948_ACCEL_XOUT_H, data, 6);

    // 16비트 값으로 조합 (High Byte << 8 | Low Byte)
    raw[0] = (int16_t)((data[0] << 8) | data[1]);  // Accel X
    raw[1] = (int16_t)((data[2] << 8) | data[3]);  // Accel Y
    raw[2] = (int16_t)((data[4] << 8) | data[5]);  // Accel Z

    // === 자이로스코프 데이터 읽기 (X, Y, Z 순서) ===
    ICM20948_ReadMultipleReg(ICM20948_GYRO_XOUT_H, data, 6);

    raw[3] = (int16_t)((data[0] << 8) | data[1]);  // Gyro X
    raw[4] = (int16_t)((data[2] << 8) | data[3]);  // Gyro Y
    raw[5] = (int16_t)((data[4] << 8) | data[5]);  // Gyro Z

    // === 센서값 스케일링 ===
    // 가속도계: LSB Sensitivity = 16384 LSB/g (±2g 범위)
    imu.accel_x = raw[0] / 16384.0f;
    imu.accel_y = raw[1] / 16384.0f;
    imu.accel_z = raw[2] / 16384.0f;

    // 자이로스코프: LSB Sensitivity = 131 LSB/(deg/s) (±250 dps 범위)
    imu.gyro_x = raw[3] / 131.0f;
    imu.gyro_y = raw[4] / 131.0f;
    imu.gyro_z = raw[5] / 131.0f;
}

/* ========================================================================
   롤/피치 각도 계산 함수
   ======================================================================== */

/**
 * @brief 가속도계 데이터로부터 롤/피치 각도 계산
 * @note atan2를 사용한 역삼각함수 계산
 */
void Calculate_Roll_Pitch(void) {
    // === 롤 각도 계산 (Y축과 Z축 가속도 사용) ===
    // Roll = atan2(ay, az) → Y축 기울기
    float raw_roll = atan2f(imu.accel_y, imu.accel_z) * 180.0f / M_PI;

    // === 피치 각도 계산 (X축과 수평면 가속도 크기 사용) ===
    // Pitch = atan2(-ax, sqrt(ay^2 + az^2)) → X축 기울기
    float raw_pitch = atan2f(-imu.accel_x,
                             sqrtf(imu.accel_y * imu.accel_y +
                                   imu.accel_z * imu.accel_z))
                      * 180.0f / M_PI;

    // === 캘리브레이션 오프셋 적용 ===
    // 초기 자세를 0도로 만들기 위해 오프셋 빼기
    imu.roll = raw_roll - roll_offset;
    imu.pitch = raw_pitch - pitch_offset;
}

/* ========================================================================
   칼만 필터 함수들
   ======================================================================== */

/**
 * @brief 칼만 필터 초기화
 * @param kf 칼만 필터 구조체 포인터
 */
void Kalman_Init(KalmanFilter *kf) {
    // 초기 각도 및 바이어스 설정
    kf->angle = 0.0f;       // 초기 각도 = 0도
    kf->bias = 0.0f;        // 초기 바이어스 = 0

    // === 오차 공분산 행렬 P 초기화 (2x2 단위 행렬) ===
    kf->P[0][0] = 1.0f;     // 각도 추정의 불확실성
    kf->P[0][1] = 0.0f;     // 상관관계
    kf->P[1][0] = 0.0f;     // 상관관계
    kf->P[1][1] = 1.0f;     // 바이어스 추정의 불확실성

    // === 노이즈 파라미터 설정 ===
    kf->Q_angle = KALMAN_Q_ANGLE;      // 프로세스 노이즈 (각도)
    kf->Q_bias = KALMAN_Q_BIAS;        // 프로세스 노이즈 (바이어스)
    kf->R_measure = KALMAN_R_MEASURE;  // 측정 노이즈
}

/**
 * @brief 칼만 필터 업데이트 - 자이로와 가속도계 데이터 융합
 * @param kf 칼만 필터 구조체 포인터
 * @param newAngle 가속도계로 측정한 새 각도 (도)
 * @param newRate 자이로스코프 각속도 (deg/s)
 * @param dt 시간 간격 (초)
 * @return 필터링된 각도 (도)
 */
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt)
{
    // ========== 1단계: PREDICT (예측 단계) ==========

    // 바이어스를 제거한 실제 각속도 계산
    // 자이로는 시간이 지나면서 드리프트(bias)가 발생하므로 보정 필요
    float rate = newRate - kf->bias;

    // 각도 예측: 현재 각도 = 이전 각도 + (각속도 × 시간)
    // 자이로 적분을 통한 각도 추정
    kf->angle += dt * rate;

    // 오차 공분산 행렬 P 업데이트 (불확실성 증가)
    // 시간이 지날수록 예측의 불확실성이 커짐
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];  // 상관관계 업데이트
    kf->P[1][0] -= dt * kf->P[1][1];  // 상관관계 업데이트
    kf->P[1][1] += kf->Q_bias * dt;   // 바이어스 불확실성 증가

    // ========== 적응형 측정 노이즈 R 계산 ==========
    // 진동이나 회전이 강할수록 가속도계의 신뢰도가 떨어지므로
    // R값을 크게 하여 가속도계 측정을 덜 신뢰함
    float motion = fabsf(newRate);              // 각속도 절댓값
    float adaptive_R = kf->R_measure * (1.0f + motion);  // 동적 R값

    // ========== 2단계: UPDATE (보정 단계) ==========

    // 혁신(Innovation) 계산: 측정값과 예측값의 차이
    float y = newAngle - kf->angle;

    // 혁신 공분산 S 계산
    // S = 예측 불확실성 + 측정 노이즈
    float S = kf->P[0][0] + adaptive_R;

    // 칼만 게인 K 계산
    // K가 클수록 측정값을 더 신뢰, 작을수록 예측값을 더 신뢰
    float K0 = kf->P[0][0] / S;  // 각도에 대한 게인
    float K1 = kf->P[1][0] / S;  // 바이어스에 대한 게인

    // 상태 보정: 예측값을 측정값으로 보정
    kf->angle += K0 * y;   // 각도 보정
    kf->bias  += K1 * y;   // 바이어스 보정 (자이로 드리프트 제거)

    // 오차 공분산 행렬 P 업데이트 (불확실성 감소)
    // 측정을 통해 불확실성이 줄어듦
    float P00 = kf->P[0][0];  // 현재 P값 백업
    float P01 = kf->P[0][1];

    kf->P[0][0] -= K0 * P00;  // P[0][0] 업데이트
    kf->P[0][1] -= K0 * P01;  // P[0][1] 업데이트
    kf->P[1][0] -= K1 * P00;  // P[1][0] 업데이트
    kf->P[1][1] -= K1 * P01;  // P[1][1] 업데이트

    // 최종 필터링된 각도 반환
    return kf->angle;
}

/* ========================================================================
   서스펜션 시스템 초기화 함수
   ======================================================================== */

/**
 * @brief 서스펜션 시스템 초기화 - 4개 서보모터 설정 및 중립 위치로 이동
 */
void Suspension_Init(void) {
    // ========== 각 서보모터에 타이머 및 채널 할당 ==========

    // 좌측 전륜 서보 → TIM2 CH1
    servos[SERVO_FRONT_LEFT].htim = &htim2;
    servos[SERVO_FRONT_LEFT].channel = TIM_CHANNEL_1;

    // 우측 전륜 서보 → TIM3 CH1
    servos[SERVO_FRONT_RIGHT].htim = &htim3;
    servos[SERVO_FRONT_RIGHT].channel = TIM_CHANNEL_1;

    // 좌측 후륜 서보 → TIM4 CH1
    servos[SERVO_REAR_LEFT].htim = &htim4;
    servos[SERVO_REAR_LEFT].channel = TIM_CHANNEL_1;

    // 우측 후륜 서보 → TIM4 CH3
    servos[SERVO_REAR_RIGHT].htim = &htim4;
    servos[SERVO_REAR_RIGHT].channel = TIM_CHANNEL_3;

    // ========== 모든 서보 PWM 시작 및 초기 위치 설정 ==========
    for(int i = 0; i < SERVO_COUNT; i++) {
        // PWM 출력 시작
        HAL_TIM_PWM_Start(servos[i].htim, servos[i].channel);

        // 초기 높이값을 중립으로 설정
        servos[i].current_height = SUSPENSION_NEUTRAL;  // 현재 높이
        servos[i].target_height = SUSPENSION_NEUTRAL;   // 목표 높이

        // 서보를 물리적으로 중립 위치로 이동
        Servo_SetHeight(i, SUSPENSION_NEUTRAL);
    }

    // 서보가 중립 위치로 이동할 시간 대기 (1초)
    HAL_Delay(1000);

    // 초기화 완료 메시지 출력
    UART_SendString("Suspension initialized to neutral\r\n");
}

/* ========================================================================
   서보모터 제어 함수들
   ======================================================================== */

/**
 * @brief 서보모터에 PWM 펄스 폭 설정
 * @param idx 서보 인덱스 (0~3)
 * @param pulse PWM 펄스 폭 (마이크로초 단위)
 */
void Servo_SetPulse(ServoIndex idx, uint16_t pulse) {
    // 타이머의 CCR(Compare Register) 값을 변경하여 PWM 듀티 설정
    __HAL_TIM_SET_COMPARE(servos[idx].htim, servos[idx].channel, pulse);
}

/**
 * @brief 서보모터를 지정된 높이(%)로 이동
 * @param idx 서보 인덱스 (0~3)
 * @param height_percent 목표 높이 (0~100%)
 * @note 작은 값일수록 서보가 더 올라감 (차량 높이 상승)
 */
void Servo_SetHeight(ServoIndex idx, uint16_t height_percent) {
    // 높이값을 허용 범위로 제한 (25% ~ 80%)
    height_percent = Constrain_u16(height_percent,
                                   SUSPENSION_MIN_HEIGHT,
                                   SUSPENSION_MAX_HEIGHT);

    // 높이 %를 PWM 펄스 폭으로 변환
    // 공식: pulse = MIN + (height% × (MAX - MIN) / 100)
    uint16_t pulse = SERVO_MIN_PULSE +
                     (height_percent * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 100;

    // PWM 출력
    Servo_SetPulse(idx, pulse);

    // 현재 높이 업데이트
    servos[idx].current_height = height_percent;
}

/* ========================================================================
   유틸리티 함수들
   ======================================================================== */

/**
 * @brief 16비트 정수값을 최소/최대 범위로 제한
 * @param value 입력값
 * @param min 최솟값
 * @param max 최댓값
 * @return 제한된 값
 */
uint16_t Constrain_u16(uint16_t value, uint16_t min, uint16_t max) {
    if (value < min) return min;  // 최솟값보다 작으면 최솟값 반환
    if (value > max) return max;  // 최댓값보다 크면 최댓값 반환
    return value;                 // 범위 내면 그대로 반환
}

/**
 * @brief 실수값을 최소/최대 범위로 제한
 * @param value 입력값
 * @param min 최솟값
 * @param max 최댓값
 * @return 제한된 값
 */
float Constrain_f(float value, float min, float max) {
    if (value < min) return min;  // 최솟값보다 작으면 최솟값 반환
    if (value > max) return max;  // 최댓값보다 크면 최댓값 반환
    return value;                 // 범위 내면 그대로 반환
}

/* ========================================================================
   IMU 캘리브레이션 함수
   ======================================================================== */

/**
 * @brief IMU 캘리브레이션 - 초기 자세를 기준점(0도)으로 설정
 * @note 평평한 곳에서 3초간 측정하여 평균값을 오프셋으로 저장
 *       이후 모든 각도 측정에서 이 오프셋을 빼서 초기 자세를 0도로 만듦
 */
void IMU_Calibrate(void) {
    char buffer[100];           // 문자열 버퍼
    float roll_sum = 0.0f;      // 롤 각도 누적값
    float pitch_sum = 0.0f;     // 피치 각도 누적값
    const int samples = 150;    // 샘플 개수 (3초 × 50Hz = 150개)

    // 캘리브레이션 시작 메시지
    UART_SendString("\r\n=== IMU Calibration Start ===\r\n");
    UART_SendString("Keep vehicle on flat surface for 3 seconds...\r\n");

    // 150번 샘플링 (3초간)
    for(int i = 0; i < samples; i++) {
        // IMU 데이터 읽기
        ICM20948_ReadAccelGyro();

        // 롤/피치 각도 계산
        Calculate_Roll_Pitch();

        // 각도값 누적
        roll_sum += imu.roll;
        pitch_sum += imu.pitch;

        // 50Hz 주기로 샘플링 (20ms 간격)
        HAL_Delay(20);

        // 50번마다 진행 상황 표시 (.)
        if(i % 50 == 0) {
            UART_SendString(".");
        }
    }

    // 평균값 계산 → 이것이 오프셋값이 됨
    roll_offset = roll_sum / samples;
    pitch_offset = pitch_sum / samples;

    // 캘리브레이션 완료 메시지
    sprintf(buffer, "\r\nCalibration complete!\r\n");
    UART_SendString(buffer);

    // 오프셋값 출력
    sprintf(buffer, "Roll offset: %.2f deg, Pitch offset: %.2f deg\r\n\r\n",
            roll_offset, pitch_offset);
    UART_SendString(buffer);
}

/* ========================================================================
   기하학적 변환 함수
   ======================================================================== */

/**
 * @brief 차체 기울기 각도를 서스펜션 높이 변화량(%)으로 변환
 * @param angle_deg 기울기 각도 (도)
 * @param distance_mm 해당 축의 거리 (wheelbase 또는 track width)
 * @return 필요한 높이 변화량 (%)
 *
 * @note 기하학적 원리:
 *       - 차체가 θ도 기울어지면 양 끝의 높이 차이 발생
 *       - 높이 차이 = 거리 × sin(θ)
 *       - 한쪽은 올리고 한쪽은 내려서 수평 유지
 */
float Angle_To_Height_Change(float angle_deg, float distance_mm) {
    // 1단계: 각도를 라디안으로 변환
    // 라디안 = 도 × (π / 180)
    float angle_rad = angle_deg * M_PI / 180.0f;

    // 2단계: 필요한 높이 차이 계산 (mm 단위)
    // 삼각함수: 높이 = 밑변 × sin(각도)
    float height_diff_mm = distance_mm * sinf(angle_rad);

    // 3단계: mm를 서보 제어 %(퍼센트)로 변환
    // HEIGHT_TO_PERCENT = 2.39 (1mm ≈ 2.39%)
    float height_percent = height_diff_mm * HEIGHT_TO_PERCENT;

    return height_percent;
}

/* ========================================================================
   PID 제어 함수
   ======================================================================== */

/**
 * @brief PID 제어기 계산 (비례-적분-미분 제어)
 * @param pid PID 제어기 구조체 포인터
 * @param error 현재 오차 (목표값 - 현재값)
 * @param dt 샘플링 시간 간격 (초)
 *
 * @note PID 제어:
 *       - P(비례): 현재 오차에 비례한 제어
 *       - I(적분): 누적 오차를 제거 (정상 상태 오차 제거)
 *       - D(미분): 오차 변화율에 대응 (오버슈트 방지)
 */
void PID_Compute(PID_Controller *pid, float error, float dt) {
    // 현재 오차값을 PID 구조체에 저장
    pid->error = error;

    // 급격한 오차 변화 감지 (충격 감지용)
    // 이전 오차와 현재 오차의 차이의 절댓값 계산
    float error_change = fabsf(error - pid->prev_error);

    // 충격 감지 시 적분항 리셋 (핵심!)
    // 오차가 15 이상 급변하면 충격(외부 교란)으로 간주
    if (error_change > 15.0f) {  // 15% 이상 급변하면 충격으로 간주
        // 적분항의 80%를 제거 (누적된 오차 리셋하여 오버슈트 방지)
        pid->integral *= 0.2f;  // 80% 제거
    }

    // I: 적분 제어 (Integral Control)
    // 데드밴드를 벗어난 오차만 적분 (미세한 오차는 무시)
    if(fabsf(error) > DEADBAND) {
        // 오차를 시간에 따라 누적 (오차 × 시간 간격)
        pid->integral += error * dt;

        // 적분항을 최대/최소 범위로 제한 (와인드업 방지)
        pid->integral = Constrain_f(pid->integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    } else {
        // 데드밴드 내부에서는 적분항을 서서히 감소 (98%만 유지)
        // 정상 상태 오차가 있을 때 적분항이 천천히 사라지도록
        pid->integral *= 0.98f;
    }

    // D: 미분 제어 (Derivative Control)
    // 오차의 변화율 계산 (현재 오차 - 이전 오차) / 시간 간격
    float raw_derivative = (error - pid->prev_error) / dt;

    // 미분항에 로우패스 필터 적용 (노이즈 제거)
    // 이전 필터값과 현재 raw 값을 가중 평균
    // DERIVATIVE_FILTER 값이 클수록 이전 값의 영향이 크고 더 부드러워짐
    pid->filtered_derivative = (DERIVATIVE_FILTER * pid->filtered_derivative) +
                               ((1.0f - DERIVATIVE_FILTER) * raw_derivative);

    // 필터링된 미분값을 최종 미분항으로 사용
    pid->derivative = pid->filtered_derivative;

    // 다음 루프를 위해 현재 오차를 이전 오차로 저장
    pid->prev_error = error;  // __error__는 오타로 보임, error가 맞음

    // PID 출력 = P + I + D
    // P항: 현재 오차에 비례 (즉각 반응)
    // I항: 누적 오차에 비례 (정상상태 오차 제거)
    // D항: 오차 변화율에 비례 (오버슈트 감소, 안정성 향상)
    pid->output = error * LEVELING_GAIN +           // P항
                  pid->integral * INTEGRAL_GAIN +    // I항
                  pid->derivative * DERIVATIVE_GAIN; // D항
}

/* ========================================================================
   서스펜션 높이 계산 함수 (PID 기반)
   ======================================================================== */

/**
 * @brief PID 제어를 사용한 실시간 4륜 독립 서스펜션 높이 계산
 * @param dt 샘플링 시간 간격 (초)
 *
 * @note 동작 원리:
 *       1. 진동 감지: 떨림이 있으면 제어 강도 감소
 *       2. 부스트 모드: 큰 기울기에서는 제어 강도 증가
 *       3. PID 계산: 롤/피치 각도를 높이 변화량으로 변환
 *       4. 스무딩: 급격한 변화를 완화하여 부드러운 동작
 */
void Calculate_Independent_Suspension_Heights_PID(float dt) {
    // ========== 1단계: 진동 감지 로직 ==========
    static float prev_roll = 0.0f, prev_pitch = 0.0f;   // 이전 각도값
    static float p_r_chg = 0.0f, p_p_chg = 0.0f;        // 이전 변화량
    static int osc_count = 0;                            // 진동 카운터

    // 현재 각도 변화량 계산
    float r_chg = imu.filtered_roll - prev_roll;    // 롤 변화량
    float p_chg = imu.filtered_pitch - prev_pitch;  // 피치 변화량

    // 방향 전환 감지 (이전 변화와 현재 변화의 부호가 다르면 진동)
    // 예: +방향 → -방향 또는 -방향 → +방향
    if((r_chg * p_r_chg < 0) || (p_chg * p_p_chg < 0)) {
        osc_count = 8;  // 진동 카운터 설정 (8회 루프 동안 댐핑)
    } else if(osc_count > 0) {
        osc_count--;    // 서서히 감소
    }

    // 다음 루프를 위해 현재값 저장
    prev_roll = imu.filtered_roll;
    prev_pitch = imu.filtered_pitch;
    p_r_chg = r_chg;
    p_p_chg = p_chg;

    // ========== 2단계: 부스트 및 댐핑 결정 ==========
    float boost = 1.0f;  // 기본 제어 강도

    // 총 기울기 계산 (롤 + 피치 절댓값)
    float total_tilt = fabsf(imu.filtered_roll) + fabsf(imu.filtered_pitch);

    // 방지턱 모드: 기울기가 3도 이상이면 파워 업
    // 큰 기울기에서는 빠르고 강하게 반응 필요
    if (total_tilt > 3.0f) {
        boost = 1.6f;  // 160% 파워
    }

    // 진동 모드: 떨림이 감지되면 파워 다운
    // 진동 시 과도한 제어는 오히려 불안정하므로 댐핑
    float osc_damping = (osc_count > 0) ? 0.4f : 1.0f;  // 40% 또는 100%

    // ========== 3단계: 기하학적 높이 변화량 계산 ==========
    // 롤 각도 → 좌우 바퀴 높이 차이
    float r_h_change = Angle_To_Height_Change(imu.filtered_roll,
                                               TRACK_WIDTH_MM / 2.0f);

    // 피치 각도 → 앞뒤 바퀴 높이 차이
    float p_h_change = Angle_To_Height_Change(imu.filtered_pitch,
                                               WHEELBASE_MM / 2.0f);

    // ========== 4단계: PID 제어 실행 ==========
    // 롤 PID 계산
    PID_Compute(&pid_roll, r_h_change, dt);

    // 피치 PID 계산
    PID_Compute(&pid_pitch, p_h_change, dt);

    // PID 출력에 부스트 및 댐핑 적용
    float r_ctrl = pid_roll.output * boost * osc_damping;   // 롤 제어량
    float p_ctrl = pid_pitch.output * boost * osc_damping;  // 피치 제어량

    // ========== 5단계: 각 바퀴의 목표 높이 계산 ==========
    // 주의: 숫자가 작을수록 차고가 높아짐!

    float target_f[4];  // 4개 바퀴의 목표 높이 배열

    // 좌측 전륜: 롤(+), 피치(-)
    // 롤 양수면 우측이 내려가므로 좌측은 올림(+r_ctrl)
    // 피치 양수면 앞이 숙여지므로 전륜은 내림(-p_ctrl)
    target_f[SERVO_FRONT_LEFT]  = (float)SUSPENSION_NEUTRAL + r_ctrl - p_ctrl;

    // 우측 전륜: 롤(-), 피치(-)
    target_f[SERVO_FRONT_RIGHT] = (float)SUSPENSION_NEUTRAL - r_ctrl - p_ctrl;

    // 좌측 후륜: 롤(+), 피치(+)
    target_f[SERVO_REAR_LEFT]   = (float)SUSPENSION_NEUTRAL + r_ctrl + p_ctrl;

    // 우측 후륜: 롤(-), 피치(+)
    target_f[SERVO_REAR_RIGHT]  = (float)SUSPENSION_NEUTRAL - r_ctrl + p_ctrl;

    // ========== 6단계: 스무딩 적용 및 저장 ==========
    for(int i = 0; i < SERVO_COUNT; i++) {
        // 높이값을 허용 범위로 제한
        float val = Constrain_f(target_f[i],
                                SUSPENSION_MIN_HEIGHT,
                                SUSPENSION_MAX_HEIGHT);

        // 가변 스무딩: 방지턱에서는 빠르게, 평소에는 부드럽게
        float alpha = (boost > 1.0f) ? 0.95f : SMOOTH_BASE;

        // 지수 이동 평균 필터 (Exponential Moving Average)
        // smoothed = α × new + (1-α) × old
        smoothed_targets[i] = (alpha * val) + ((1.0f - alpha) * smoothed_targets[i]);

        // 정수로 반올림하여 저장
        servos[i].target_height = (uint16_t)(smoothed_targets[i] + 0.5f);
    }
}

/* ========================================================================
   서보 점진적 이동 함수
   ======================================================================== */

/**
 * @brief 서보를 목표 높이로 점진적으로 이동
 * @note 급격한 움직임을 방지하고 부드러운 동작 구현
 *       간격(gap)에 따라 이동 속도를 다르게 설정
 */
void Update_Servos_Gradually(void) {
    // 모든 서보 모터를 순회하며 점진적으로 위치 업데이트
    for(int i = 0; i < SERVO_COUNT; i++) {
        // 목표 높이와 현재 높이의 차이(갭) 계산
        float gap = (float)servos[i].target_height - servos[i].current_height;

        // 각 서보의 이전 목표값을 저장하는 정적 배열 (초기값: 30)
        static float prev_targets[4] = {30, 30, 30, 30};

        // 이전 목표값과 현재 목표값의 변화량 계산 (절댓값)
        float target_change = fabsf(servos[i].target_height - prev_targets[i]);

        // 스텝 게인 변수 선언 (이동 속도를 결정)
        float step_gain;

        // 목표값이 급격히 변경된 경우 (8 이상 변화)
        if(target_change > 8.0f) {
            step_gain = 0.02f;  // 매우 느리게 이동 (급격한 변화 방지)
        }
        // 갭이 20 초과: 매우 멀리 떨어진 경우
        else if(fabsf(gap) > 20.0f) {
            step_gain = 0.15f;  // 빠르게 이동
        }
        // 갭이 15 초과 20 이하: 꽤 멀리 떨어진 경우
        else if(fabsf(gap) > 15.0f) {
            step_gain = 0.12f;  // 중간-빠른 속도
        }
        // 갭이 10 초과 15 이하: 중간 거리
        else if(fabsf(gap) > 10.0f) {
            step_gain = 0.09f;  // 중간 속도
        }
        // 갭이 5 초과 10 이하: 가까운 거리
        else if(fabsf(gap) > 5.0f) {
            step_gain = 0.06f;  // 느린 속도
        }
        // 갭이 3 초과 5 이하: 매우 가까운 거리
        else if(fabsf(gap) > 3.0f) {
            step_gain = 0.04f;  // 더 느린 속도
        }
        // 갭이 1 초과 3 이하: 거의 도착
        else if(fabsf(gap) > 1.0f) {
            step_gain = 0.03f;  // 매우 느린 속도
        }
        // 갭이 1 이하: 거의 목표 도달
        else {
            step_gain = 0.05f;  // 미세 조정
        }

        // 실제 이동 스텝 계산: 갭 × 게인 = 이동량
        float step = gap * step_gain;

        // 갭이 0.05보다 큰 경우 (아직 목표에 도달하지 않음)
        if (fabsf(gap) > 0.05f) {
            // 현재 높이에 스텝만큼 추가 (점진적 이동)
            servos[i].current_height += step;
        } else {
            // 갭이 매우 작으면 목표값으로 직접 설정 (떨림 방지)
            servos[i].current_height = (float)servos[i].target_height;
        }

        // 다음 루프를 위해 현재 목표값을 이전 목표값으로 저장
        prev_targets[i] = servos[i].target_height;

        // 계산된 현재 높이를 정수로 반올림하여 실제 서보에 적용
        // (+0.5f는 반올림을 위한 상수)
        Servo_SetHeight(i, (uint16_t)(servos[i].current_height + 0.5f));
    }
}

/* ========================================================================
   서스펜션 리셋 함수
   ======================================================================== */

/**
 * @brief 서스펜션을 중립 위치로 복귀
 * @note CAN 통신으로 서스펜션이 OFF될 때 호출됨
 */
void Reset_Suspension_To_Neutral(void) {
    UART_SendString("Resetting suspension to neutral position...\r\n");

    // ========== PID 제어기 초기화 ==========
    // 적분 항 초기화 (누적된 오차 제거)
    pid_roll.integral = 0.0f;
    pid_roll.prev_error = 0.0f;
    pid_roll.output = 0.0f;

    pid_pitch.integral = 0.0f;
    pid_pitch.prev_error = 0.0f;
    pid_pitch.output = 0.0f;

    // ========== 모든 서보를 중립 위치로 설정 ==========
    for(int i = 0; i < SERVO_COUNT; i++) {
        servos[i].target_height = SUSPENSION_NEUTRAL;   // 목표 높이 = 중립
        servos[i].current_height = SUSPENSION_NEUTRAL;  // 현재 높이 = 중립
        smoothed_targets[i] = SUSPENSION_NEUTRAL;       // 스무딩 버퍼 = 중립

        // 서보를 물리적으로 중립 위치로 이동
        Servo_SetHeight(i, SUSPENSION_NEUTRAL);
    }
}

/* ========================================================================
   CAN 통신 초기화 함수
   ======================================================================== */

/**
 * @brief CAN 통신 초기화 및 필터 설정
 * @note ID 0x10 메시지만 수신하도록 필터 설정
 *       0x10: 서스펜션 ON/OFF 제어 명령
 */
void CAN_App_Init(void)
{
    // CAN 필터 구조체 선언 및 초기화
    CAN_FilterTypeDef filter = {0};

    // ========== CAN 필터 설정 ==========
    filter.FilterBank = 0;                          // 필터 뱅크 번호 (0~13 중 선택)
    filter.FilterMode = CAN_FILTERMODE_IDMASK;      // ID 마스크 모드 (ID와 마스크로 필터링)
    filter.FilterScale = CAN_FILTERSCALE_32BIT;     // 32비트 필터 스케일
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0에 메시지 저장
    filter.FilterActivation = ENABLE;                // 필터 활성화

    // ========== 필터 ID 및 마스크 설정 ==========
    // 0x10 ID만 수신하도록 설정
    filter.FilterIdHigh     = (0x10 << 5);   // 필터 ID 상위 비트 (0x10을 5비트 왼쪽 시프트)
    filter.FilterIdLow      = 0x0000;         // 필터 ID 하위 비트
    filter.FilterMaskIdHigh = (0x7FF << 5);  // 마스크 상위 비트 (11비트 모두 체크)
    filter.FilterMaskIdLow  = 0x0000;         // 마스크 하위 비트

    // ========== CAN 시작 및 인터럽트 활성화 ==========
    // 필터 설정 적용
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        Error_Handler();  // 실패 시 에러 핸들러 호출

    // CAN 통신 시작
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();  // 실패 시 에러 핸들러 호출

    // FIFO0 수신 인터럽트 활성화
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();  // 실패 시 에러 핸들러 호출

    // 초기화 완료 메시지 출력
    UART_SendString("CAN initialized - Listening on ID 0x10\r\n");
}

/* ========================================================================
   CAN 수신 인터럽트 콜백 함수
   ======================================================================== */

/**
 * @brief CAN 메시지 수신 시 자동으로 호출되는 콜백 함수
 * @param hcan CAN 핸들 포인터
 * @note 이 함수는 CAN 메시지가 수신되면 하드웨어 인터럽트에 의해 자동 호출됨
 *       ID 0x10 메시지로 서스펜션 ON/OFF 제어
 *       rxData[0] = 0: OFF, 1: ON
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // CAN 수신 헤더 구조체
    CAN_RxHeaderTypeDef rxHeader;

    // 수신 데이터 버퍼 (최대 8바이트)
    uint8_t rxData[8];

    // ========== 수신 메시지 검증 ==========
    // CAN1이 아니면 무시
    if (hcan->Instance != CAN1) return;

    // FIFO0에서 메시지 읽기
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;  // 읽기 실패 시 종료

    // 표준 ID (11비트)가 아니면 무시
    if (rxHeader.IDE != CAN_ID_STD) return;

    // 데이터 길이가 1바이트 미만이면 무시
    if (rxHeader.DLC < 1) return;

    // ========== ID 0x10: 서스펜션 ON/OFF 제어 ==========
    if (rxHeader.StdId == 0x10) {
        // 이전 서스펜션 상태 저장 (상태 변경 감지용)
        uint8_t prev_state = g_suspension_enabled;

        // 데이터 유효성 검사 (0 또는 1만 허용)
        if (rxData[0] <= 1) {
            // 전역 변수에 새로운 상태 저장
            g_suspension_enabled = rxData[0];

            // 상태가 실제로 변경되었을 때만 메시지 출력
            if (prev_state != g_suspension_enabled) {
                // 서스펜션이 켜진 경우
                if (g_suspension_enabled) {
                    UART_SendString("\r\n>>> Suspension Control ENABLED <<<\r\n");
                }
                // 서스펜션이 꺼진 경우
                else {
                    UART_SendString("\r\n>>> Suspension Control DISABLED <<<\r\n");

                    // OFF될 때 서스펜션을 안전하게 중립으로 복귀
                    Reset_Suspension_To_Neutral();
                }
            }
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

  /* USER CODE BEGIN 1 */
  // UART 출력용 문자열 버퍼 (256바이트)
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

  /* ========================================================================
     시스템 시작 메시지 출력
     ======================================================================== */

  UART_SendString("\r\n\r\n");  // 줄바꿈으로 구분
  UART_SendString("=====================================\r\n");
  UART_SendString("  ICM-20948 + 4x Servo Active Suspension\r\n");
  UART_SendString("  PID Auto-Leveling System\r\n");
  UART_SendString("=====================================\r\n\r\n");

  // 시스템 안정화를 위한 대기 시간
  HAL_Delay(100);

  /* ========================================================================
     ICM-20948 IMU 센서 초기화
     ======================================================================== */

  if (ICM20948_Init() == 0) {
      // 초기화 성공
      UART_SendString("ICM-20948 Init Success!\r\n");
  } else {
      // 초기화 실패 → 시스템 정지
      UART_SendString("ICM-20948 Init Failed!\r\n");
      while(1);  // 무한 루프 (더 이상 진행 불가)
  }

  /* ========================================================================
     서스펜션 시스템 초기화
     ======================================================================== */

  UART_SendString("Initializing Suspension System...\r\n");

  // 4개 서보모터 초기화 및 중립 위치로 이동
  Suspension_Init();

  UART_SendString("System Ready!\r\n\r\n");

  /* ========================================================================
     IMU 캘리브레이션
     ======================================================================== */

  // 평평한 곳에서 3초간 측정하여 초기 자세를 기준점(0도)으로 설정
  IMU_Calibrate();

  /* ========================================================================
     칼만 필터 초기화
     ======================================================================== */

  // 롤 각도용 칼만 필터 초기화
  Kalman_Init(&kalman_roll);

  // 피치 각도용 칼만 필터 초기화
  Kalman_Init(&kalman_pitch);

  // 추가 안정화 대기
  HAL_Delay(500);

  /* ========================================================================
     제어 루프 준비
     ======================================================================== */

  // ICM-20948을 Bank 0으로 설정 (센서 데이터 읽기 모드)
  ICM20948_SelectBank(0);

  // 마지막 업데이트 시각 초기화 (HAL_GetTick()은 시스템 시작 후 ms 반환)
  last_update_time = HAL_GetTick();

  // 루프 카운터 초기화 (제어 주파수 측정용)
  uint32_t loop_count = 0;

  // 마지막 출력 시각 초기화 (500ms마다 상태 출력용)
  uint32_t last_print_time = HAL_GetTick();

  // 제어 시작 메시지
  UART_SendString("Starting active suspension control...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // ========== 현재 시각 읽기 ==========
      // HAL_GetTick()은 시스템 시작 후 경과한 밀리초를 반환
      uint32_t current_time = HAL_GetTick();

      /* ====================================================================
         100Hz 제어 루프 (10ms마다 실행)
         ==================================================================== */

      // 마지막 업데이트로부터 10ms 이상 경과했는지 확인
      if (current_time - last_update_time >= UPDATE_RATE_MS) {
          // ========== 실제 경과 시간 계산 ==========
          // dt = 경과 시간(ms) / 1000 → 초 단위로 변환
          float dt = (current_time - last_update_time) / 1000.0f;

          // 다음 루프를 위해 현재 시각 저장
          last_update_time = current_time;

          /* ================================================================
             IMU 데이터 읽기 및 처리
             ================================================================ */

          // 1단계: 가속도계 + 자이로스코프 원시 데이터 읽기
          ICM20948_ReadAccelGyro();

          // 2단계: 가속도계 데이터로 롤/피치 각도 계산
          Calculate_Roll_Pitch();

          // 3단계: 칼만 필터 적용 (자이로 + 가속도계 데이터 융합)
          // 롤 각도 필터링: 가속도계 각도 + 자이로 각속도 + 시간 간격
          imu.filtered_roll = Kalman_Update(&kalman_roll,
                                            imu.roll,      // 가속도계 롤 각도
                                            imu.gyro_x,    // 자이로 X축 각속도
                                            dt);           // 시간 간격

          // 피치 각도 필터링: 가속도계 각도 + 자이로 각속도 + 시간 간격
          imu.filtered_pitch = Kalman_Update(&kalman_pitch,
                                             imu.pitch,     // 가속도계 피치 각도
                                             imu.gyro_y,    // 자이로 Y축 각속도
                                             dt);           // 시간 간격

          /* ================================================================
             서스펜션 제어 (CAN 명령에 의해 활성화된 경우만)
             ================================================================ */

          // g_suspension_enabled가 1일 때만 서스펜션 제어 수행
          // (CAN 메시지로 ON/OFF 제어됨)
          if (g_suspension_enabled) {
              // 4-1. PID 제어를 통한 각 바퀴의 목표 높이 계산
              Calculate_Independent_Suspension_Heights_PID(dt);

              // 4-2. 서보를 목표 높이로 점진적으로 이동
              Update_Servos_Gradually();
          }

          // 루프 카운터 증가 (주파수 측정용)
          loop_count++;
      }

      /* ====================================================================
         500ms마다 상태 데이터 출력 (디버깅용)
         ==================================================================== */

      // 마지막 출력으로부터 500ms 이상 경과했는지 확인
      if (current_time - last_print_time >= 500) {
          // 다음 출력을 위해 현재 시각 저장
          last_print_time = current_time;

          // ========== 헤더 출력 ==========
          UART_SendString("\r\n=== PID Control Status ===\r\n");

          // ========== 현재 롤/피치 각도 출력 ==========
          // sprintf: 형식 문자열을 buffer에 저장
          sprintf(buffer, "Roll: %.2f deg, Pitch: %.2f deg\r\n",
                  imu.filtered_roll,   // 칼만 필터 적용된 롤 각도
                  imu.filtered_pitch); // 칼만 필터 적용된 피치 각도
          UART_SendString(buffer);

          // ========== 서스펜션 상태에 따른 출력 분기 ==========
          if (g_suspension_enabled) {
              // ----- 서스펜션 활성화 상태: 상세 정보 출력 -----

              // PID 피치 제어 상세 정보 (P, I, D 항 개별 출력)
              sprintf(buffer, "PID Pitch - P:%.2f I:%.2f D:%.2f Out:%.2f\r\n",
                      pid_pitch.error * LEVELING_GAIN,      // P항: 비례 제어
                      pid_pitch.integral * INTEGRAL_GAIN,   // I항: 적분 제어
                      pid_pitch.derivative * DERIVATIVE_GAIN, // D항: 미분 제어
                      pid_pitch.output);                     // 총 출력
              UART_SendString(buffer);

              // 4개 바퀴의 현재 높이 출력 (% 단위)
              sprintf(buffer, "Wheel Heights (%%): FL=%.1f FR=%.1f RL=%.1f RR=%.1f\r\n",
                      servos[SERVO_FRONT_LEFT].current_height,   // 좌측 전륜
                      servos[SERVO_FRONT_RIGHT].current_height,  // 우측 전륜
                      servos[SERVO_REAR_LEFT].current_height,    // 좌측 후륜
                      servos[SERVO_REAR_RIGHT].current_height);  // 우측 후륜
              UART_SendString(buffer);
          }
          else {
              // ----- 서스펜션 비활성화 상태: 중립 위치 표시 -----
              UART_SendString("Suspension at NEUTRAL position\r\n");
          }

          // ========== 제어 루프 주파수 출력 ==========
          // loop_count는 500ms 동안 누적된 루프 횟수
          // 1초당 루프 횟수 = loop_count × 2
          sprintf(buffer, "Loop Rate: %lu Hz\r\n", loop_count * 2);
          UART_SendString(buffer);

          // 다음 측정을 위해 카운터 리셋
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
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
