//main.c file 
// rest is generated from stm32 cube mx

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* ================= USER DEFINES ================= */

#define BH1750_ADDR1   (0x23 << 1)   // ADDR pin = GND
#define BH1750_ADDR2   (0x5C << 1)   // ADDR pin = VCC

/* ================= PERIPHERAL HANDLES ================= */

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* ================= FUNCTION PROTOTYPES ================= */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* ================= BH1750 FUNCTIONS ================= */

/* Check if BH1750 responds on I2C */
uint8_t BH1750_Check(uint16_t addr)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, addr, 3, 100) == HAL_OK);
}

/* Initialize BH1750 */
void BH1750_Init(uint16_t addr)
{
    uint8_t cmd;

    if (!BH1750_Check(addr)) return;

    cmd = 0x01;  // Power ON
    HAL_I2C_Master_Transmit(&hi2c1, addr, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    cmd = 0x07;  // Reset
    HAL_I2C_Master_Transmit(&hi2c1, addr, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    cmd = 0x10;  // Continuous High Resolution Mode
    HAL_I2C_Master_Transmit(&hi2c1, addr, &cmd, 1, HAL_MAX_DELAY);

    HAL_Delay(180); // Measurement time
}

/* Read lux value */
float BH1750_ReadLux(uint16_t addr)
{
    uint8_t data[2];
    uint16_t raw;

    if (!BH1750_Check(addr))
        return -1.0f;

    if (HAL_I2C_Master_Receive(&hi2c1, addr, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return -1.0f;

    raw = (data[0] << 8) | data[1];
    return raw / 1.2f;
}

/* ================= MAIN ================= */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    /* Initialize both BH1750 sensors */
    BH1750_Init(BH1750_ADDR1);
    BH1750_Init(BH1750_ADDR2);

    char msg[100];
    float lux1, lux2;

    while (1)
    {
        lux1 = BH1750_ReadLux(BH1750_ADDR1);
        HAL_Delay(180);

        lux2 = BH1750_ReadLux(BH1750_ADDR2);
        HAL_Delay(180);

        sprintf(msg,
                "BH1750_1: %.2f lx | BH1750_2: %.2f lx\r\n",
                lux1, lux2);

        HAL_UART_Transmit(&huart2,
                          (uint8_t*)msg,
                          strlen(msg),
                          HAL_MAX_DELAY);

        HAL_Delay(500);
    }
}

/* ================= I2C INIT ================= */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10D19CE4;   // 100kHz @ 80MHz
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

/* ================= UART INIT ================= */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

/* ================= GPIO INIT ================= */

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* ================= CLOCK CONFIG ================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

/* ================= ERROR HANDLER ================= */

void Error_Handler(void)
{
    __disable_irq();
    while (1);
}
