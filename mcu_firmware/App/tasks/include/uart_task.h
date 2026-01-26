#ifndef UART_TASK_H
#define UART_TASK_H

#include "main.h"          // for UART handle & HAL includes


// Task function prototype
void StartUartTask(void *argument);
void uart_printf(const char *fmt, ...);

extern UART_HandleTypeDef huart2;
#endif /* ROS_TASK_H */
