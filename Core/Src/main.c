#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "pca9685.h"

static void MX_GPIO_Init(void);
void SystemClock_Config(void);

/* ============ ????:STM32 ???/???? ============ */
// ????(???? 60°)
static float curDeg[3] = {60.0f, 60.0f, 60.0f};
static float tgtDeg[3] = {60.0f, 60.0f, 60.0f};

// ??:?/?(???????;?? 20~80 ???)
#define SERVO_SPEED_DPS   40.0f

// ????:20ms -> 50Hz
#define SERVO_UPDATE_MS   20u

// usart.c ???????????:??????
void OnServoCommand(uint8_t servoId, uint16_t angle)
{
  if (servoId < 1 || servoId > 3) return;
  if (angle > 180) angle = 180;
  tgtDeg[servoId - 1] = (float)angle;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  USART1_StartRxLine();

  // ??????,?????“?????”
  HAL_Delay(300);

  // ? PCA9685 ready(?? 3 ?)
  uint32_t t0 = HAL_GetTick();
  while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(PCA9685_I2C_ADDR_7BIT << 1), 2, 100) != HAL_OK)
  {
    if (HAL_GetTick() - t0 > 3000)
      Error_Handler();
    HAL_Delay(50);
  }

  // ??? PCA9685 50Hz
  if (PCA9685_Init50Hz(PCA9685_I2C_ADDR_7BIT) != HAL_OK)
    Error_Handler();

  // ?????????????(??? 60°)
  for (uint8_t i = 0; i < 3; i++)
  {
    PCA9685_ServoWriteDeg(PCA9685_I2C_ADDR_7BIT, i, (uint16_t)(curDeg[i] + 0.5f));
  }

  // ???:?? 50Hz ? current ??? target
  uint32_t last = HAL_GetTick();

  while (1)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last) >= SERVO_UPDATE_MS)
    {
      last += SERVO_UPDATE_MS;

      // ???????????:speed * dt
      float step = SERVO_SPEED_DPS * ((float)SERVO_UPDATE_MS / 1000.0f); // ?? 40dps * 0.02 = 0.8°

      for (uint8_t i = 0; i < 3; i++)
      {
        float diff = tgtDeg[i] - curDeg[i];

        if (diff > step)       curDeg[i] += step;
        else if (diff < -step) curDeg[i] -= step;
        else                   curDeg[i]  = tgtDeg[i];

        // ?? PCA:????????
        PCA9685_ServoWriteDeg(PCA9685_I2C_ADDR_7BIT, i, (uint16_t)(curDeg[i] + 0.5f));
      }
    }

    // ? CPU ????(??)
    HAL_Delay(1);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  // ?? LED ????
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_Delay(150);
  }
}
