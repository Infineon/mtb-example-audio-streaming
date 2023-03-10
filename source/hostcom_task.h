/******************************************************************************
* File Name: hostcom_task.h
*
* Description: This file is the public interface of hostcom_task.c source file.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef SOURCE_HOSTCOM_TASK_H_
#define SOURCE_HOSTCOM_TASK_H_

/*******************************************************************************
* Header files includes
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern TaskHandle_t xTaskHostCom;
extern QueueHandle_t cobs_q;

/*******************************************************************************
* Structures and enumerations
*******************************************************************************/
typedef struct _cobs_msg {
    uint8_t  *msg_buf;
    uint16_t len;
} cobs_msg_t;

typedef struct _cobs_buf {
    uint8_t bytes[0x1fc]; // the payload 1:255 and ONE trailing 0x00 byte
    uint32_t len; // num of payload bytes incl the trailing 0x00 byte
} cobs_buf_t;

/*******************************************************************************
* Function prototypes
*******************************************************************************/
void task_hostcom(void* param);
void task_uart_tx(void* param);

#endif /* SOURCE_HOSTCOM_TASK_H_ */

/* [] END OF FILE */
