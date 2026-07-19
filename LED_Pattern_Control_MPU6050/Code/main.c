/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU6050 LED Bar Graph - Multi-Mode Creative Display
  * @description    : 4 Modes - Normal, Chase, Wave, Flash (Shake to switch)
  ******************************************************************************
  * Modes:
  * MODE 1: Normal Bar Graph - LEDs light up based on tilt
  * MODE 2: Knight Rider Chase - Single LED chases back and forth
  * MODE 3: Wave Effect - Multiple LEDs create wave pattern
  * MODE 4: Flash Effect - LEDs flash based on tilt direction
  *
  * Shake the sensor to switch between modes!
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    MODE_NORMAL = 0,   // Bar graph
    MODE_CHASE,        // Knight Rider
    MODE_WAVE,         // Wave effect
    MODE_FLASH,        // Flash effect
    MODE_COUNT         // Total modes
} DisplayMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6050_ADDR 0xD0
#define SHAKE_DEBOUNCE_TIME 2000  // Milliseconds between mode changes
#define SHAKE_THRESHOLD 0.6       // G-force change to detect shake

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

// MPU6050 Raw Data
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

// MPU6050 Processed Data
float Ax, Ay, Az;
float prev_Az = 0;

// Status Variables
uint8_t check = 0;
uint8_t mpu_init_success = 0;

// Mode Control
DisplayMode current_mode = MODE_NORMAL;
uint32_t mode_change_time = 0;
uint32_t last_shake_time = 0;        // Track last shake time for debouncing
uint8_t shake_cooldown_active = 0;   // Prevent multiple triggers

// Animation Variables
uint8_t chase_position = 4;
int8_t chase_direction = 1;
uint32_t last_chase_time = 0;

uint8_t wave_offset = 0;
uint32_t last_wave_time = 0;

uint32_t flash_time = 0;
uint8_t flash_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

// MPU6050 Functions
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);

// Display Functions
void Turn_Off_All_LEDs(void);
void Display_With_Mode(float tiltValue);

// Mode Functions
void Mode_Normal(float tiltValue);
void Mode_Chase(float tiltValue);
void Mode_Wave(float tiltValue);
void Mode_Flash(float tiltValue);

// Utility Functions
uint8_t Detect_Shake(void);
void Show_Mode_Indicator(void);
void Startup_Animation(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
   MPU6050 INITIALIZATION
   ============================================================================ */
void MPU6050_Init(void)
{
    uint8_t Data;
    HAL_StatusTypeDef status;

    HAL_Delay(100);

    // Check device ID
    status = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);

    if (status != HAL_OK || check != 0x68)
    {
        mpu_init_success = 0;
        return;
    }

    mpu_init_success = 1;

    // Wake up sensor
    Data = 0;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, 1, &Data, 1, 1000);
    HAL_Delay(10);

    // Set sample rate
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x19, 1, &Data, 1, 1000);

    // Configure gyro (not used but good to set)
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1B, 1, &Data, 1, 1000);

    // Configure accelerometer ± 2g
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1C, 1, &Data, 1, 1000);

    HAL_Delay(50);
}

/* ============================================================================
   MPU6050 READ ACCELEROMETER
   ============================================================================ */
void MPU6050_Read_Accel(void)
{
    uint8_t Rec_Data[6];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

    if (status != HAL_OK) {
        return;
    }

    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convert to 'g' values
    Ax = (float)Accel_X_RAW / 16384.0;
    Ay = (float)Accel_Y_RAW / 16384.0;
    Az = (float)Accel_Z_RAW / 16384.0;
}

/* ============================================================================
   SHAKE DETECTION - WITH DEBOUNCING (FIXED)
   ============================================================================ */
uint8_t Detect_Shake(void)
{
    uint32_t current_time = HAL_GetTick();

    // Calculate change in Z-axis acceleration
    float az_change = Az - prev_Az;
    prev_Az = Az;

    // Debounce: Ignore shakes within SHAKE_DEBOUNCE_TIME of last shake
    if (current_time - last_shake_time < SHAKE_DEBOUNCE_TIME) {
        return 0;  // Too soon, ignore
    }

    // Detect sudden Z-axis movement (shake)
    if (az_change > SHAKE_THRESHOLD || az_change < -SHAKE_THRESHOLD) {
        last_shake_time = current_time;  // Record shake time
        return 1;  // Shake detected!
    }

    return 0;  // No shake
}

/* ============================================================================
   LED CONTROL - TURN OFF ALL
   ============================================================================ */
void Turn_Off_All_LEDs(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
                             GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
}

/* ============================================================================
   MODE 1: NORMAL BAR GRAPH
   ============================================================================ */
void Mode_Normal(float tiltValue)
{
    Turn_Off_All_LEDs();

    float threshold1 = 0.15;
    float threshold2 = 0.30;
    float threshold3 = 0.50;
    float threshold4 = 0.70;

    if (tiltValue > threshold1)
    {
        // Tilting RIGHT
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

        if (tiltValue > threshold2) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
        if (tiltValue > threshold3) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        }
        if (tiltValue > threshold4) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }
    }
    else if (tiltValue < -threshold1)
    {
        // Tilting LEFT
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

        if (tiltValue < -threshold2) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        }
        if (tiltValue < -threshold3) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        }
        if (tiltValue < -threshold4) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        }
    }
}

/* ============================================================================
   MODE 2: KNIGHT RIDER CHASE EFFECT
   ============================================================================ */
void Mode_Chase(float tiltValue)
{
    uint32_t current_time = HAL_GetTick();

    // Speed based on tilt magnitude
    uint16_t chase_speed = 100;
    float absT = tiltValue > 0 ? tiltValue : -tiltValue;

    if (absT > 0.5) chase_speed = 30;
    else if (absT > 0.3) chase_speed = 50;
    else if (absT > 0.15) chase_speed = 70;

    // Update position
    if (current_time - last_chase_time > chase_speed)
    {
        last_chase_time = current_time;

        // Direction based on tilt
        if (tiltValue > 0.1) chase_direction = 1;
        else if (tiltValue < -0.1) chase_direction = -1;

        chase_position += chase_direction;

        // Bounce at edges
        if (chase_position >= 7) {
            chase_position = 7;
            chase_direction = -1;
        }
        if (chase_position == 0) {
            chase_position = 0;
            chase_direction = 1;
        }
    }

    // Display
    Turn_Off_All_LEDs();

    switch(chase_position)
    {
        case 0: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); break;
        case 3: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); break;
        case 4: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); break;
        case 5: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); break;
        case 6: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); break;
        case 7: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); break;
    }
}

/* ============================================================================
   MODE 3: WAVE EFFECT
   ============================================================================ */
void Mode_Wave(float tiltValue)
{
    uint32_t current_time = HAL_GetTick();

    // Update wave position
    if (current_time - last_wave_time > 80) {
        last_wave_time = current_time;
        wave_offset++;
        if (wave_offset >= 8) wave_offset = 0;
    }

    Turn_Off_All_LEDs();

    // Number of active LEDs based on tilt
    uint8_t num_leds = 1;
    float absT = tiltValue > 0 ? tiltValue : -tiltValue;

    if (absT > 0.5) num_leds = 4;
    else if (absT > 0.3) num_leds = 3;
    else if (absT > 0.15) num_leds = 2;

    // Light up LEDs in wave pattern
    for (uint8_t i = 0; i < num_leds; i++) {
        uint8_t pos = (wave_offset + i) % 8;

        switch(pos) {
            case 0: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); break;
            case 1: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); break;
            case 2: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); break;
            case 3: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); break;
            case 4: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); break;
            case 5: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); break;
            case 6: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); break;
            case 7: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); break;
        }
    }
}

/* ============================================================================
   MODE 4: FLASH EFFECT
   ============================================================================ */
void Mode_Flash(float tiltValue)
{
    // Flash speed based on tilt
    uint16_t flash_speed = 500;
    float absT = tiltValue > 0 ? tiltValue : -tiltValue;

    if (absT > 0.5) flash_speed = 80;
    else if (absT > 0.3) flash_speed = 150;
    else if (absT > 0.15) flash_speed = 250;

    if (HAL_GetTick() - flash_time > flash_speed) {
        flash_time = HAL_GetTick();
        flash_state = !flash_state;

        if (flash_state) {
            // Flash direction based on tilt
            if (tiltValue > 0.05) {
                // Right side
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
            }
            else if (tiltValue < -0.05) {
                // Left side
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
            }
            else {
                // All when centered
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
                                         GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
            }
        } else {
            Turn_Off_All_LEDs();
        }
    }
}

/* ============================================================================
   MODE INDICATOR - SHOWS WHICH MODE IS ACTIVE
   ============================================================================ */
void Show_Mode_Indicator(void)
{
    Turn_Off_All_LEDs();

    // Blink number of times = mode number + 1
    // MODE 0 = 1 blink, MODE 1 = 2 blinks, etc.
    for (uint8_t i = 0; i <= current_mode; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(150);
        Turn_Off_All_LEDs();
        HAL_Delay(150);
    }
    HAL_Delay(300);
}

/* ============================================================================
   STARTUP ANIMATION
   ============================================================================ */
void Startup_Animation(void)
{
    // Left to Right sweep
    for (uint8_t i = 0; i < 8; i++) {
        Turn_Off_All_LEDs();
        HAL_GPIO_WritePin(GPIOA, 1 << i, GPIO_PIN_SET);
        HAL_Delay(80);
    }

    // Right to Left sweep
    for (int8_t i = 7; i >= 0; i--) {
        Turn_Off_All_LEDs();
        HAL_GPIO_WritePin(GPIOA, 1 << i, GPIO_PIN_SET);
        HAL_Delay(80);
    }

    // All on then off
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
                             GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(300);
    Turn_Off_All_LEDs();
    HAL_Delay(200);
}

/* ============================================================================
   MAIN DISPLAY FUNCTION WITH MODE SWITCHING - DEBOUNCED
   ============================================================================ */
void Display_With_Mode(float tiltValue)
{
    uint32_t current_time = HAL_GetTick();

    // Only check for shake if not in cooldown
    if (!shake_cooldown_active && Detect_Shake()) {
        // Activate cooldown to prevent processing during indicator display
        shake_cooldown_active = 1;

        // Switch to next mode
        current_mode = (current_mode + 1) % MODE_COUNT;
        mode_change_time = current_time;

        // Show which mode we switched to
        Show_Mode_Indicator();

        // Reset animation variables for clean mode start
        chase_position = 4;
        wave_offset = 0;
        flash_state = 0;

        // Deactivate cooldown after indicator shown
        shake_cooldown_active = 0;

        // Update last shake time to prevent immediate re-trigger
        last_shake_time = HAL_GetTick();
    }

    // Execute current mode
    switch(current_mode) {
        case MODE_NORMAL:
            Mode_Normal(tiltValue);
            break;

        case MODE_CHASE:
            Mode_Chase(tiltValue);
            break;

        case MODE_WAVE:
            Mode_Wave(tiltValue);
            break;

        case MODE_FLASH:
            Mode_Flash(tiltValue);
            break;

        default:
            Mode_Normal(tiltValue);
            break;
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
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  // Initialize MPU6050
  MPU6050_Init();

  // Check initialization
  if (mpu_init_success == 0) {
      // Error: Blink all LEDs rapidly
      while(1) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
                                   GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
          HAL_Delay(200);
          Turn_Off_All_LEDs();
          HAL_Delay(200);
      }
  }

  // Success! Show startup animation
  Startup_Animation();

  // Initialize accelerometer tracking variables
  MPU6050_Read_Accel();
  prev_Az = Az;
  last_shake_time = HAL_GetTick();  // Initialize shake timer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Read sensor data
    MPU6050_Read_Accel();

    // Display with current mode
    Display_With_Mode(Ay);  // Using Y-axis for left-right tilt

    // Update rate - 50Hz (20ms delay)
    HAL_Delay(20);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
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
