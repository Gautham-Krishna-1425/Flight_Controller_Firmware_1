/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application entry point
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "attitude_controller.h"

#include "mpu9250.h"
#include "bmp280.h"
#include <math.h>
#include "motor_mixer.h"
#include "motor_pwm.h"
#include "ibus.h"
#include "ppm.h"

#define DISABLE_MOTORS 0
#define HITL_MODE 0
#define ESC_CALIBRATION_MODE 0
#define ENABLE_PRINT 0


#define IWDG_REFRESH()  do { \
    IWDG->KR = 0xAAAA; \
} while(0)

#define IWDG_START()  do { \
    IWDG->KR  = 0x5555; \
    IWDG->PR  = 0x02;   /* prescaler /32 → 1kHz */ \
    IWDG->RLR = 250;    /* 250ms timeout */ \
    IWDG->KR  = 0xCCCC; /* start */ \
} while(0)


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    PPM_InputCaptureCallback(htim);
}


#define DEG2RAD 0.01745329251f   // pi/180
#define GYRO_EMA_ALPHA 0.35f

#include "gps.h"
#include "vertical_kf.h"
#include "altitude_pid.h"


MotorOutput_t motors;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum
{
    IMU_NONE = 0,
    IMU_MPU6050,
    IMU_MPU9250
} imu_type_t;

static imu_type_t active_imu = IMU_NONE;

uint8_t gps_rx;
uint8_t ibus_rx;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;



/* USER CODE BEGIN PV */
VerticalKF_t vert_kf;
AltPID_t alt_pid;

float desired_alt = 1.0f;   // 1 meter hover
float throttle_cmd = 0.5f;

float baro_offset = 0.0f;
float az_bias = 0.0f;


float gyro_bias_roll = 0.0f;
float gyro_bias_pitch = 0.0f;
float gyro_bias_yaw = 0.0f;


float gyro_roll_filt  = 0.0f;
float gyro_pitch_filt = 0.0f;
float gyro_yaw_filt   = 0.0f;

AttitudeAxis_t roll_ctrl;
AttitudeAxis_t pitch_ctrl;

uint32_t last_rc_time = 0;

uint8_t esp_rx_buf = 0;
uint8_t fan_active = 0;

static float roll_f  = 0.0f;
static float pitch_f = 0.0f;

static uint8_t angle_f_init = 0;

static uint8_t esp_cmd_byte = 0;


/* Altitude control variables */
float desired_altitude = 0.0f;


uint32_t lastTick = 0;

float ax, ay, az;
float gx, gy, gz;
float roll, pitch, yaw;


float roll_trim  = 0.0f;
float pitch_trim = 0.0f;


float temperature, pressure, altitude;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);


static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

typedef enum { DISARMED, ARMING, ARMED } arm_state_t;
static arm_state_t arm_state = DISARMED;
static uint32_t arm_timer = 0;

static inline uint16_t rc_map(uint16_t v)
{
    static uint16_t v_min = 65535;
    static uint16_t v_max = 0;

    // Track observed range
    if (v < v_min) v_min = v;
    if (v > v_max) v_max = v;

    // Prevent divide-by-zero during startup
    if (v_max <= v_min + 10)
        return 1500;

    // Map to 1000–2000
    uint32_t out = 1000 + (uint32_t)(v - v_min) * 1000 / (v_max - v_min);

    if (out < 1000) out = 1000;
    if (out > 2000) out = 2000;

    return (uint16_t)out;
}



/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Redirect printf to UART */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, 10);
    return len;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();


  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  GPS_Init();

#if ESC_CALIBRATION_MODE
    // Start PWM hardware manually — skip the 1000us arming delay
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    // Send 2000us immediately — before battery connects
    MotorPWM_Write(1.0f, 1.0f, 1.0f, 1.0f);
    printf("ESC CAL: Connect battery now!\r\n");

    HAL_Delay(6000);   // 6 seconds — connect battery during this window

    // Drop to 1000us — ESCs confirm calibration
    MotorPWM_Write(0.0f, 0.0f, 0.0f, 0.0f);
    printf("ESC CAL: Dropped to low. Listen for confirm beeps\r\n");

    HAL_Delay(4000);   // wait for ESC confirmation sequence
    printf("ESC CAL: Done. Set ESC_CALIBRATION_MODE=0 and reflash\r\n");
    while(1);          // stop here — forces reflash before flying
#else
    MotorPWM_Init();
#endif

  IBUS_Init();
  HAL_UART_Receive_IT(&huart2, &ibus_rx, 1);

  last_rc_time = HAL_GetTick();


  /* USER CODE BEGIN 2 */

  /* Scan I2C bus (optional but useful) */
  I2C_Scan(&hi2c1);

  /* Initialize MPU6050 */
  if (MPU6050_Init(&hi2c1) != MPU_OK)
  {
      printf("MPU6050 not detected ❌\r\n");
      while (1);   // hard fail – IMU is mandatory
  }

  printf("MPU6050 detected ✅\r\n");


  /* Initialize BMP280 */
  if (bmp280_init(&hi2c1) != HAL_OK)
  {
  printf("BMP280 not detected ❌\r\n");
  while (1);
  }
  printf("BMP280 detected ✅\r\n");


  float baro_ref = 0.0f;


  bmp280_read_temp_press(&hi2c1, 0.01f,
                         &temperature, &pressure,
                         &baro_ref,
                         NULL,
                         NULL);

  float baro_zero = 0.0f;




  // Average a few samples
  for (int i = 0; i < 50; i++) {
      bmp280_read_temp_press(&hi2c1, 0.02f,
                             &temperature, &pressure,
                             &baro_zero,
                             NULL, NULL);
      HAL_Delay(20);
  }

  baro_offset = baro_zero;




  HAL_UART_Receive_IT(&huart1, &gps_rx, 1);
  HAL_Delay(1000);

  if(GPS_IsDetected())
      printf("GPS detected ✅\r\n");
  else
      printf("GPS not detected ❌\r\n");



  /* IMPORTANT: keep board perfectly still here */
  active_imu = IMU_MPU6050;

  HAL_Delay(200);

  /* Gyro calibration */
  MPU6050_CalibrateGyro(&hi2c1);
  printf("Gyro calibrated ✅\r\n");


  // ===== LEVEL CALIBRATION =====
  // Let the filter settle first
  float level_roll_offset  = 0.0f;
  float level_pitch_offset = 0.0f;

  printf("Leveling... keep drone flat\r\n");
  HAL_Delay(2000);  // let baro and gyro settle

  // Run the filter for 200 loops to let it converge

  for (int i = 0; i < 500; i++)
  {
      MPU6050_RawData_t imu_cal;
      MPU6050_ReadRaw(&hi2c1, &imu_cal);
      float r=0, p=0, y_=0;
      MPU6050_ComputeAngles(imu_cal.ax, imu_cal.ay, imu_cal.az,
                            imu_cal.gx, imu_cal.gy, imu_cal.gz,
                            0.005f, &r, &p, &y_);
      if (i >= 200) {
          level_roll_offset  += r;
          level_pitch_offset += p;
      }
      HAL_Delay(5);
  }

  level_roll_offset  /= 300.0f;
  level_pitch_offset /= 300.0f;

  printf("Level offsets: R=%.2f P=%.2f\r\n",
         level_roll_offset, level_pitch_offset);


  // ===== ACCEL Z CALIBRATION =====
  float az_sum = 0.0f;
  for (int i = 0; i < 200; i++) {
      MPU6050_RawData_t imu;
      MPU6050_ReadRaw(&hi2c1, &imu);
      az_sum += imu.az;
      HAL_Delay(5);
  }

  // Expected az ≈ -16384 when Z-up (because you inverted sign)
  az_bias = az_sum / 200.0f;

  printf("Accel Z bias = %.1f\n", az_bias);



  roll_ctrl.K_po  = 0.8f;    // was 3.0 (too aggressive) / 0.15 (too weak)
  pitch_ctrl.K_po = 0.8f;

  roll_ctrl.K_pi  = 0.035f;  // was 0.08 / 0.010
  pitch_ctrl.K_pi = 0.035f;

  roll_ctrl.K_di  = 0.0018f; // was 0.003 / 0.0012
  pitch_ctrl.K_di = 0.0018f;

  roll_ctrl.K_ii  = 0.030f;  // was 0.06 / 0.020
  pitch_ctrl.K_ii = 0.030f;


  Attitude_Init(&pitch_ctrl);
  Attitude_Init(&roll_ctrl);


  roll_ctrl.alpha  = 0.015f;
  pitch_ctrl.alpha = 0.015f;

  VerticalKF_Init(&vert_kf);

  vert_kf.h  = 0.0f;
  vert_kf.vz = 0.0f;


  alt_pid.kp = 1.2f;
  alt_pid.ki = 0.3f;
  alt_pid.kd = 0.6f;

  alt_pid.out_min = -0.3f;
  alt_pid.out_max =  0.3f;
  AltPID_Init(&alt_pid);


  HAL_UART_Receive_IT(&huart6, &esp_rx_buf, 1);



  /* USER CODE END 2 */
  lastTick = HAL_GetTick();




  // before while(1)
  float rc_roll = 0.0f;
  float rc_pitch = 0.0f;
  float rc_yaw = 0.0f;
  float rc_throttle = 0.0f;

  static uint8_t is_airborne = 0;



  uint32_t prevLoopStart = HAL_GetTick();

  IWDG_START();







  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)

 {




      // ===== Time step =====
	  uint32_t loopStart = HAL_GetTick();

	  IWDG_REFRESH();



	  if ((loopStart - prevLoopStart) < 5) continue;

	  float dt = (loopStart - prevLoopStart) / 1000.0f;
	  prevLoopStart = loopStart;
	  if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;




	  // ===== HARD RC FAILSAFE =====
	  if ((HAL_GetTick() - last_rc_time) > 500)
	  {
	      arm_state = DISARMED;

	      is_airborne  = 0;

	      roll_ctrl.integrator = 0;
	      pitch_ctrl.integrator = 0;

	      roll_ctrl.prev_rate_meas = 0;
	      pitch_ctrl.prev_rate_meas = 0;

	      roll_ctrl.d_filt = 0;
	      pitch_ctrl.d_filt = 0;

	      throttle_cmd = 0.0f;




	      MotorPWM_Write(0,0,0,0);

	      prevLoopStart = HAL_GetTick();

	      continue;   // EXIT LOOP IMMEDIATELY
	  }


      // ===== Read IMU ===== //
	  MPU6050_RawData_t imu = {0};
	  if (MPU6050_ReadRaw(&hi2c1, &imu) == MPU_OK)
	  {
	      MPU6050_ComputeAngles(
	          imu.ax, imu.ay, imu.az,
	          imu.gx, imu.gy, imu.gz,
	          dt, &roll, &pitch, &yaw
	      );
	      // Move these INSIDE:
	      ax = imu.ax;
	      ay = imu.ay;
	      az = imu.az;
	      gx = imu.gx;
	      gy = imu.gy;
	      gz = imu.gz;
	  }
	  else
	  {
	      // I2C failed — attempt recovery
	      HAL_I2C_DeInit(&hi2c1);
	      HAL_Delay(1);
	      MX_I2C1_Init();   // reinit bus
	      // Don't update angles — keep last known values for this cycle
	  }


	  pitch -= level_pitch_offset;
	  roll  -= level_roll_offset;

	  if (!angle_f_init) {
	      roll_f  = roll;
	      pitch_f = pitch;
	      angle_f_init = 1;
	  }


      roll_f  += 0.35f * (roll  - roll_f);   // ~10Hz LPF
      pitch_f += 0.35f * (pitch - pitch_f);







      // ===== Read BMP280 =====
	  static float baro_alt = 0.0f;
	  static uint32_t baro_last = 0;
	  if (HAL_GetTick() - baro_last >= 40)
	  {
	      baro_last = HAL_GetTick();
	      bmp280_read_temp_press(
	          &hi2c1, dt, &temperature,
	          &pressure, &baro_alt, NULL, NULL
	      );
	  }


      // ===== BARO LOW-PASS FILTER =====
      static float baro_f = 0.0f;
      static uint8_t baro_f_init = 0;

      float baro_h = baro_alt - baro_offset;

      if (!baro_f_init) {
          baro_f = baro_h;
          vert_kf.h  = baro_f;
          vert_kf.vz = 0.0f;
          baro_f_init = 1;
      }


      // 1st-order LPF (cutoff ~2 Hz @ 100 Hz loop)
      baro_f += 0.05f * (baro_h - baro_f);



      uint16_t ch0 = IBUS_GetChannel(0);
      uint16_t ch1 = IBUS_GetChannel(1);
      uint16_t ch2 = IBUS_GetChannel(2);
      uint16_t ch3 = IBUS_GetChannel(3);

      if (ch0 >= 900 && ch0 <= 2100 &&
          ch1 >= 900 && ch1 <= 2100 &&
          ch2 >= 900 && ch2 <= 2100 &&
          ch3 >= 900 && ch3 <= 2100)
      {
    	  last_rc_time = HAL_GetTick();
          rc_roll     = (ch0 - 1500) / 500.0f;
          rc_pitch    = (ch1 - 1500) / 500.0f;
          rc_yaw      = (ch3 - 1500) / 500.0f;
          rc_throttle = (ch2 - 1000) / 1000.0f;
      }
      else
      {
          rc_roll     = 0.0f;
          rc_pitch    = 0.0f;
          rc_yaw      = 0.0f;
          rc_throttle = 0.0f;
      }

      if (fabsf(rc_roll)  < 0.05f) rc_roll  = 0.0f;
      if (fabsf(rc_pitch) < 0.05f) rc_pitch = 0.0f;
      if (fabsf(rc_yaw)   < 0.05f) rc_yaw   = 0.0f;



      // ===== Cascaded attitude controller =====



      float desired_roll  = rc_roll  * 20.0f + roll_trim;
      float desired_pitch = rc_pitch * 20.0f + pitch_trim;


      float gyro_roll_raw  = (imu.gx - gyro_x_offset) / 131.0f;
      float gyro_pitch_raw = (imu.gy - gyro_y_offset) / 131.0f;
      float gyro_yaw_raw   = (imu.gz - gyro_z_offset) / 131.0f;

      gyro_roll_filt  += GYRO_EMA_ALPHA * (gyro_roll_raw  - gyro_roll_filt);
      gyro_pitch_filt += GYRO_EMA_ALPHA * (gyro_pitch_raw - gyro_pitch_filt);
      gyro_yaw_filt   += GYRO_EMA_ALPHA * (gyro_yaw_raw   - gyro_yaw_filt);

      float gyro_roll_dps  = gyro_roll_filt;
      float gyro_pitch_dps = gyro_pitch_filt;
      float gyro_yaw_dps   = gyro_yaw_filt;

      static uint8_t was_airborne = 0;
      static uint8_t takeoff_warmup = 0;

      float ctrl_roll  = roll_f;
      float ctrl_pitch = pitch_f;

      // Transition: reset everything + start warmup window
      if (is_airborne && !was_airborne) {
          roll_ctrl.integrator      = 0;
          pitch_ctrl.integrator     = 0;
          roll_ctrl.d_filt          = 0;
          pitch_ctrl.d_filt         = 0;
          roll_ctrl.prev_rate_meas  = 0;
          pitch_ctrl.prev_rate_meas = 0;
          takeoff_warmup = 60;
      }
      was_airborne = is_airborne;

      // Zero control inputs while grounded OR during warmup window
      if (!is_airborne || takeoff_warmup > 0) {
          gyro_roll_dps  = 0.0f;
          gyro_pitch_dps = 0.0f;
          ctrl_roll  = 0.0f;
          ctrl_pitch = 0.0f;
          if (takeoff_warmup > 0) takeoff_warmup--;
      }


      float u_roll = Attitude_Update(&roll_ctrl,
                         desired_roll,
                         ctrl_roll,
                         gyro_roll_dps, dt);


      float u_pitch = Attitude_Update(&pitch_ctrl,
                          desired_pitch,
                          ctrl_pitch,     // deadbanded
                          gyro_pitch_dps, dt);

      if (is_airborne &&
          takeoff_warmup == 0 &&
          fabsf(rc_roll) < 0.05f &&
          fabsf(rc_pitch) < 0.05f)
      {
    	  roll_trim  += roll_ctrl.integrator  * 0.001f;
    	  pitch_trim += pitch_ctrl.integrator * 0.001f;
      }

      if (roll_trim > 5) roll_trim = 5;
      if (roll_trim < -5) roll_trim = -5;

      if (pitch_trim > 5) pitch_trim = 5;
      if (pitch_trim < -5) pitch_trim = -5;




	// ===== Altitude control =====


	// Convert raw accel to m/s² (sensor frame, Z-up)
	float az_corr   = az - az_bias;          // bias-corrected, ≈0 at rest
	float az_mps2   = (az_corr / 16384.0f) * 9.81f;   // no sign flip needed
	float acc_z_world = az_mps2 * cosf(roll * DEG2RAD) * cosf(pitch * DEG2RAD);
	// No gravity subtraction — az_bias already removes gravity

	// ---- ACCEL SANITY FILTER (CRITICAL) ----

	// Deadband small noise
	if (fabsf(acc_z_world) < 0.2f)
	    acc_z_world = 0.0f;

	// Clamp to realistic vertical accel (hand motion safe)
	if (acc_z_world >  3.0f) acc_z_world =  3.0f;
	if (acc_z_world < -3.0f) acc_z_world = -3.0f;





	// If vertical motion is small, trust baro only
	if (fabsf(acc_z_world) < 0.5f && fabsf(vert_kf.vz) < 0.5f) {
	    acc_z_world = 0.0f;
	}


	if (throttle_cmd < 0.1f) {
	    vert_kf.vz *= 0.9f;   // bleed velocity on ground
	}




	// ===== ARMING STATE MACHINE =====
	uint16_t arm_ch2 = IBUS_GetChannel(2);   // throttle channel
	uint16_t arm_ch3 = IBUS_GetChannel(3);   // yaw channel

	// Valid IBUS signal is always between 1000-2000
	// If it's 0 or garbage, don't allow arming
	bool throttle_low = (arm_ch2 > 900 && arm_ch2 < 1150);
	bool yaw_right = (arm_ch3 > 1800 && arm_ch3 < 2100);
	static uint32_t disarm_timer = 0;





	if (arm_state == DISARMED) {

	    // Wait until RC has been valid for at least 1 second
	    // before allowing any arming gesture
	    static uint32_t rc_valid_since = 0;
	    static uint8_t rc_was_valid = 0;

	    bool rc_valid = (arm_ch2 > 900 && arm_ch2 < 2100 &&
	                     arm_ch3 > 900 && arm_ch3 < 2100);

	    if (rc_valid && !rc_was_valid) {
	        // RC just became valid — record time
	        rc_valid_since = HAL_GetTick();
	        rc_was_valid = 1;
	    }

	    if (!rc_valid) {
	        // RC lost — reset everything
	        rc_was_valid = 0;
	        rc_valid_since = 0;
	        arm_timer = 0;
	    }

	    // Only allow arming gesture after RC stable for 1 second
	    bool rc_stable = rc_was_valid &&
	                    (HAL_GetTick() - rc_valid_since > 1000);

	    if (rc_stable && throttle_low && yaw_right) {

	        if (arm_timer == 0) {
	            arm_timer = HAL_GetTick();
	        }

	        if (HAL_GetTick() - arm_timer > 2000) {
	            arm_state = ARMED;
	            arm_timer = 0;
	            printf("ARMED\r\n");
	        }

	    } else {
	        arm_timer = 0;
	    }
	}
	else if (arm_state == ARMED) {

		uint16_t disarm_ch3 = IBUS_GetChannel(3);
		bool yaw_left = (disarm_ch3 > 900 && disarm_ch3 < 1200);

		if (throttle_low && yaw_left)
 {

	        if (disarm_timer == 0) {
	            disarm_timer = HAL_GetTick();
	        }

	        if (HAL_GetTick() - disarm_timer > 1000) {
	            arm_state = DISARMED;
	            disarm_timer = 0;
	            printf("DISARMED\r\n");
	        }

	    } else {
	        disarm_timer = 0;
	    }

	}



	// ===== RC INPUT =====

	if (rc_throttle < 0.0f) rc_throttle = 0.0f;
	if (rc_throttle > 1.0f) rc_throttle = 1.0f;

	bool alt_hold = false;
	static bool alt_hold_prev = false;

	// ===== ALT HOLD EDGE DETECT =====
	if (alt_hold && !alt_hold_prev) {
	    desired_alt = vert_kf.h;
	    AltPID_Reset(&alt_pid);
	}
	alt_hold_prev = alt_hold;

	// ===== THROTTLE COMMAND =====
	float hover_throttle = 0.50f;
	float u_alt = 0.0f;

	if (alt_hold) {
		u_alt = AltPID_Update(&alt_pid,
	                                desired_alt,
	                                vert_kf.h,
	                                vert_kf.vz,
	                                dt);
	    throttle_cmd = hover_throttle + u_alt;
	} else {
	    throttle_cmd = rc_throttle;
	}

	if (throttle_cmd < 0.0f) throttle_cmd = 0.0f;
	if (throttle_cmd > 1.0f) throttle_cmd = 1.0f;

	// ===== THRUST MODEL =====
	float thrust_accel = (throttle_cmd - hover_throttle) * 8.0f;

	if (throttle_cmd < 0.1f) {
	    thrust_accel = 0.0f;
	}

	// ===== VERTICAL KF =====
	#if HITL_MODE
	    VerticalKF_Update(&vert_kf,
	                      thrust_accel,
	                      baro_f,
	                      dt);
	#else
	    VerticalKF_Update(&vert_kf,
	                      acc_z_world,
	                      baro_f,
	                      dt);
	#endif


	// ===== VZ ZERO-VELOCITY UPDATE (ZUPT) =====
	if (fabsf(vert_kf.vz) < 0.03f && fabsf(baro_h - baro_f) < 0.02f)
	{
	    vert_kf.vz = 0.0f;
	}




	// Altitude controller




	if (throttle_cmd < 0.0f) throttle_cmd = 0.0f;
	if (throttle_cmd > 1.0f) throttle_cmd = 1.0f;



	static uint32_t arm_settled_time = 0;
		if (arm_state == ARMED && arm_settled_time == 0)
		    arm_settled_time = HAL_GetTick();
		if (arm_state == DISARMED)
		    arm_settled_time = 0;
		if (arm_settled_time && (HAL_GetTick() - arm_settled_time < 500))
		    rc_yaw = 0.0f;


		// ===== YAW CONTROL =====
		#define K_YAW_P         0.08f    // start here, increase if yaw feels sluggish
		#define TARGET_YAW_RATE 100.0f   // deg/s at full stick deflection


		if (arm_state == ARMED && arm_settled_time == 0)
		    arm_settled_time = HAL_GetTick();
		if (arm_state == DISARMED)
		    arm_settled_time = 0;
		if (arm_settled_time && (HAL_GetTick() - arm_settled_time < 500))
		    rc_yaw = 0.0f;

		float yaw_rate_setpoint = rc_yaw * TARGET_YAW_RATE;   // deg/s
		float u_yaw = (yaw_rate_setpoint - gyro_yaw_dps) * K_YAW_P;

		// Clamp yaw authority so it doesn't steal too much from throttle
		if (u_yaw >  0.15f) u_yaw =  0.15f;
		if (u_yaw < -0.15f) u_yaw = -0.15f;




	// ===== HARD DISARM BLOCK =====
	if (arm_state != ARMED)
	{
	    roll_ctrl.integrator = 0;
	    pitch_ctrl.integrator = 0;
	    is_airborne           = 0;
	    MotorPWM_Write(0,0,0,0);
	    prevLoopStart = HAL_GetTick();
	    continue;
	}

	// ===== THROTTLE GATE =====
	// Silent when armed but throttle not raised yet

	if (throttle_cmd > 0.45f) is_airborne = 1;
	if (arm_state == DISARMED)  is_airborne = 0;

	#define THROTTLE_SPINUP_THRESHOLD 0.10f
	#define MOTOR_INFLIGHT_MIN        0.05f

	if (throttle_cmd < THROTTLE_SPINUP_THRESHOLD && !is_airborne)
	{
	    // Armed, on ground, throttle not raised — stay silent
	    MotorPWM_Write(0, 0, 0, 0);
	    roll_ctrl.integrator     = 0;
	    pitch_ctrl.integrator    = 0;
	    roll_ctrl.d_filt         = 0;
	    pitch_ctrl.d_filt        = 0;
	    roll_ctrl.prev_rate_meas = 0;
	    pitch_ctrl.prev_rate_meas = 0;
	    prevLoopStart = HAL_GetTick();
	    continue;
	}



	// Throttle is raised — run mixer
	MotorMixer_X(throttle_cmd,
	             u_roll,
	             -u_pitch,
	             u_yaw,
	             &motors);


	// In-flight floor — keeps ESCs alive once airborne
	if (throttle_cmd > THROTTLE_SPINUP_THRESHOLD) {
	    motors.m1 = fmaxf(motors.m1, 0.05f);
	    motors.m2 = fmaxf(motors.m2, 0.05f);
	    motors.m3 = fmaxf(motors.m3, 0.05f);
	    motors.m4 = fmaxf(motors.m4, 0.05f);
	}

	#if DISABLE_MOTORS
	    MotorPWM_Write(0, 0, 0, 0);
	#else
	    if (arm_state == ARMED)
	        MotorPWM_Write(motors.m1, motors.m2, motors.m3, motors.m4);
	    else
	        MotorPWM_Write(0, 0, 0, 0);
	#endif





	/*static GPS_Data_t g;          // keep last known GPS
	if (GPS_GetData(&g))          // only updates when a sentence is parsed
	{
	   printf("GPS FIX %d SAT=%d LAT=%.6f LON=%.6f SPD=%.2f ALT=%.1f\r\n",
	         g.fix, g.satellites,
	         g.latitude, g.longitude,
	         g.speed_mps, g.altitude);
	}

*/

/*
	    printf("<ATT=%.2f,%.2f,%.2f|"
	    	       "RC=%.2f,%.2f,%.2f,%.2f|"
	    			"M=%.2f,%.2f,%.2f,%.2f>\n",
	    	       roll, pitch, yaw,
	    	       rc_roll, rc_pitch, rc_throttle, rc_yaw,
	    	       motors.m1, motors.m2, motors.m3, motors.m4);


*/



/*


	static uint32_t last_print = 0;
	if (HAL_GetTick() - last_print >= 100)
	{   // print at 10Hz max
	    last_print = HAL_GetTick();
	    printf("<R=%.2f P=%.2f DR=%.2f DP=%.2f M1=%.2f M2=%.2f M3=%.2f M4=%.2f>\n",
	           roll, pitch,
	           desired_roll, desired_pitch,
	           motors.m1, motors.m2, motors.m3, motors.m4);
	}
	*/

#if ENABLE_PRINT
	/*static uint32_t last_print = 0;
	  if (HAL_GetTick() - last_print >= 100)
	  {
	      last_print = HAL_GetTick();
	      printf("R%.1f P%.1f UR%.3f UP%.3f M%.2f %.2f %.2f %.2f\n",
	             roll, pitch, u_roll, u_pitch,
	             motors.m1, motors.m2, motors.m3, motors.m4);

	  }
	  */

	/*  static uint32_t last_print = 0;
	  if (HAL_GetTick() - last_print >= 20)
	  {
	      last_print = HAL_GetTick();
	      printf("roll:%.2f,pitch:%.2f,M1:%.2f,M2:%.2f,M3:%.2f,M4:%.2f\n",
	             roll, pitch,
	             motors.m1, motors.m2, motors.m3, motors.m4);
	  } */



	  static uint32_t last_print = 0;
	  if (HAL_GetTick() - last_print >= 20)
	  {
	      last_print = HAL_GetTick();
	      printf("des_roll:%.2f,des_pitch:%.2f,M1:%.2f,M2:%.2f,M3:%.2f,M4:%.2f\n",
	             desired_roll, desired_pitch,
	             motors.m1, motors.m2, motors.m3, motors.m4);
	  }


#endif


  }






    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
	__HAL_RCC_TIM1_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    __HAL_RCC_TIM1_CLK_ENABLE();


    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 83;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    // PWM configuration
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000;  // start at 1000us
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // CH1 (PA8 / PB13N)
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    // CH2 (PB14N)
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    // CH3 (PB15N)
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();

    // Enable advanced timer output
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
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
  htim2.Init.Prescaler = 83;
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
  sConfigOC.Pulse = 1000;
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
  HAL_TIM_MspPostInit(&htim2);


}
  /* USER CODE BEGIN TIM2_Init 2 */


  /* USER CODE END TIM2_Init 2 */
static void MX_TIM3_Init(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (SystemCoreClock / 1000000) - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_IC_Init(&htim3);

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;

    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
}


static void MX_TIM4_Init(void)
{
	__HAL_RCC_TIM4_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 83;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 19999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}




/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
