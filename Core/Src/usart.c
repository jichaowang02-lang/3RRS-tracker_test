#include "usart.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>


UART_HandleTypeDef huart1;

static uint8_t rx_ch;
static char line_buf[32];
static volatile uint8_t line_len = 0;

// main.c 中实现
extern void OnServoCommand(uint8_t servoId, uint16_t angle);

static void parse_line(const char *s) {
  // 协议格式: S1:090 / S2:045 / S3:180
  if (s[0] != 'S')
    return;
  if (s[1] < '1' || s[1] > '3')
    return;
  if (s[2] != ':')
    return;

  int angle = atoi(&s[3]);
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;

  OnServoCommand((uint8_t)(s[1] - '0'), (uint16_t)angle);
}

void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

void USART1_StartRxLine(void) {
  line_len = 0;
  HAL_UART_Receive_IT(&huart1, &rx_ch, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1)
    return;

  char c = (char)rx_ch;

  if (c == '\n' || c == '\r') {
    if (line_len > 0) {
      line_buf[line_len] = '\0';
      parse_line(line_buf);
      line_len = 0;
    }
  } else {
    if (line_len < (uint8_t)(sizeof(line_buf) - 1))
      line_buf[line_len++] = c;
    else
      line_len = 0;
  }

  HAL_UART_Receive_IT(&huart1, &rx_ch, 1);
}
