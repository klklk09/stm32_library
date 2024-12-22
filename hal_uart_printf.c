#include <stdio.h>
#include <stdarg.h>
#include <string.h>
// 重定�? printf 函数
void UART_Printf(const char *format, ...)
{
  char buffer[100];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

void UART_Print(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}