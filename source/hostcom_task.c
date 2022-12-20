/*******************************************************************************
* File Name:   hostcom_task.c
*
* Description: This file contains the communication task to send the data to 
*              the host.
*
* Related Document: See README.md
*
*
********************************************************************************
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
* Header files includes
*******************************************************************************/
#include "hostcom_task.h"
#include "cybsp.h"
#include "cyhal.h"
#include "FreeRTOS.h"
#include "queue.h"

/* everything we need for encoding/decoding PB messages which we
 * want to send to host or receive from host 
 */
#include "pb_decode.h"
#include "pb_encode.h"
#include "speech_sample.pb.h"

/* everything we need to add packet boundaries to those packets we
 * need to transfer over a com channel which does NOT have frame 
 * sync capabilities
 */
#include "cobs.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Define how many samples in a frame */
/* 128 samples @ 16kHz => 8ms */
/* 256 samples @ 16kHz => 16ms */
#define FRAME_SIZE                  (128)
/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ              (16000u)
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             (48u)
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          (24576000u)
/* Size of proto buffer for encoding */
#define PB_BUFFER_SIZE              (1024)
/* Size of buffer storing speech samples */
#define SPEECH_BUFFER_SIZE          (4)
/* Size of buffer storing cobs messages */
#define COBS_BUFFER_SIZE            (4)
/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

#define BAUD_RATE                   (1000000)
#define INT_PRIORITY                (3)
#define DATA_BITS_8                 (8)
#define STOP_BITS_1                 (1)

/*******************************************************************************
* Global constants
*******************************************************************************/

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = 
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_RIGHT, 
    .word_length     = 16,  /* bits */
    .left_gain       = 0,   /* dB */
    .right_gain      = 21,   /* dB */
};

/* Initialize the UART configuration structure */
const cyhal_uart_cfg_t uart_config =
{
    .data_bits      = DATA_BITS_8,
    .stop_bits      = STOP_BITS_1,
    .parity         = CYHAL_UART_PARITY_NONE,
    .rx_buffer      = NULL,
    .rx_buffer_size = 0
};

/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;

/* Variable Declarations */
cyhal_uart_t uart_obj;

TaskHandle_t xTaskHostCom;
QueueHandle_t cobs_q;

uint8_t pb_encode_buf[PB_BUFFER_SIZE];

pb_ostream_t stream;

SpeechSample speech[SPEECH_BUFFER_SIZE];

/* read/write idx for the frames of audio samples.
 * msg_wr_idx is operated by the ISR handler (more general: The writer task)
 * whereas msg_rd_idx is operated by the PB-encoding task (more general: The reader task).
 * This is a typical/classical reader/writer pattern.
 */
uint8_t msg_wr_idx = 0;
uint8_t msg_rd_idx = 0;

uint8_t cobs_wr_idx = 0;

cyhal_uart_event_t last_event;

cobs_buf_t cobs_bufs[COBS_BUFFER_SIZE];

uint32_t pkts_encoded = 0;

/*******************************************************************************
* Function Name: pdm_pcm_clock_init
********************************************************************************
* Summary:
*  This function configures clock signals for the audio subsystem.
*
*  A helpful block diagram of PSoC6 clock hierarchy is found in the
*  mtb-hal-cat1 documentation: mtb_shared/mtb-hal-cat1/release-v2.2.0/docs/html/
*                              group__group__hal__impl__clock.html
*******************************************************************************/
void pdm_pcm_clock_init(void)
{
    cy_rslt_t result;

    /* Initialize the PLL clock (CYHAL_CLOCK_PLL[0])
     * See above mentioned diagram and cyhal_clock.c for more details. */
    result = cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    if (result != CY_RSLT_SUCCESS) CY_ASSERT(0);

    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1]) 
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    result = cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
    if (result != CY_RSLT_SUCCESS) CY_ASSERT(0);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

/*******************************************************************************
 * Function Name: pdm_pcm_isr_handler
 ********************************************************************************
 * Summary:
 *  This function handles CYHAL_PDM_PCM_ASYNC_COMPLETE event once FRAME_SIZE 
 *  samples have arrived in audio buffer and want to be processed. 
 *
 * Parameters:
 *  arg: void* param
 *  event: event that occurred
 *
 *******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cyhal_pdm_pcm_t *pdm_pcm = (cyhal_pdm_pcm_t*) arg;

    if (0u != (event & CYHAL_PDM_PCM_ASYNC_COMPLETE)) {
        msg_wr_idx += 1;
        msg_wr_idx &= 3;
        cyhal_pdm_pcm_read_async(pdm_pcm, &speech[msg_wr_idx].samples, FRAME_SIZE);

        /* dispatch a notification to xTaskHostCom, query scheduler whether we need to 
        return to interrupted task or whether such notification re-shuffled the list of ready-to-run tasks. 
        If so, we return to 'new' highest prio task from here.*/
        vTaskNotifyGiveFromISR(xTaskHostCom, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*******************************************************************************
 * Function Name: uart_isr_handler
 ********************************************************************************
 * Summary:
 *  This functon handles CYHAL_UART_IRQ_TX_DONE event once one package is put 
 *  in the UART TX
 *
 * Parameters:
 *  arg: cyhal_pdm_pcm_t*
 *  event: event that occurred
 *
 *******************************************************************************/
void uart_isr_handler(void *arg, cyhal_uart_event_t event) {
    size_t tx_len;
    cobs_buf_t* cobs_buf_ptr;

    if (0u != (event & CYHAL_UART_IRQ_TX_DONE)) {
        /* if the writer thread generated a new msg into the q, 
         * we pop and send it.
         */
        if (xQueueReceiveFromISR(cobs_q, &cobs_buf_ptr, NULL) == pdTRUE) {
            tx_len = cobs_buf_ptr->len +1;
            cyhal_uart_write_async(&uart_obj, cobs_buf_ptr->bytes, tx_len);
        }
    }
    else {
        last_event |= event;
    }
}

/*******************************************************************************
 * Function Name: task_hostcom
 ********************************************************************************
 * Summary:
 *  This function is the communication task to send the data to the host. 
 *
 * Parameters:
 *  arg: void* param
 *
 *******************************************************************************/
void task_hostcom(void* param)
{
    cy_rslt_t result;
    size_t tx_len = 0;
    BaseType_t q_post;
    uint32_t actual_baud;
    cobs_encode_result cobs_enc_rslt;

    /* Configure the clock tree for the audio subsystem (=> I2S, PDM-PCM peripheral) */
    pdm_pcm_clock_init();
    result = cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    /* Enable PDM/PCM event. ISR is called when ASYNC transfer is complete, transfer is done by DMA. */
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, &pdm_pcm);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    result = cyhal_pdm_pcm_set_async_mode(&pdm_pcm, CYHAL_ASYNC_DMA, CYHAL_DMA_PRIORITY_DEFAULT);
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    /* Initialize the UART Block */
    result = cyhal_uart_init(&uart_obj, CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, NC, NC, NULL,
                           &uart_config);
    /* Set the baud rate */
    result = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actual_baud);

    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }
    /* The UART callback handler registration */
    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_uart_register_callback(&uart_obj, uart_isr_handler, NULL);
        /* Enable required UART events */
        cyhal_uart_enable_event(&uart_obj,
                                (cyhal_uart_event_t)(CYHAL_UART_IRQ_TX_DONE),
                                INT_PRIORITY, true);
    }

    /* Start the PDM-PCM peripheral */ 
    cyhal_pdm_pcm_clear(&pdm_pcm);
    cyhal_pdm_pcm_start(&pdm_pcm);
    /* note that this triggers async transfer of audio into 'speech[]'. */
    /* Once complete, the ISR is triggered which in turn triggers the next async transfer. */
    cyhal_pdm_pcm_read_async(&pdm_pcm, &speech[0].samples, FRAME_SIZE);

    for(;;)
    {
        /* we suspend until we get notified that a new chunk of audio data has arrived in 'speech[]'. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* we have new data in 'speech[]', work with it: 
         * - add a little meta-data to the members of 'speech'
         * - serialize 'speech' into a PB packet
         */
        stream = pb_ostream_from_buffer(pb_encode_buf, sizeof(pb_encode_buf));

        speech[msg_rd_idx].timestamp = xTaskGetTickCountFromISR();
        speech[msg_rd_idx].pkts_encoded = pkts_encoded;
        
        if (!pb_encode(&stream, SpeechSample_fields, &speech[msg_rd_idx])) {
            CY_ASSERT(0);
        }
        msg_rd_idx += 1; msg_rd_idx &= 3;
        pkts_encoded +=1;

        /* Encode the byte stream (made of values in 0:255) into a 
         * COBS-encoded byte stream (with values 1:255) and add delimiting byte (0x00)
         * at the end.
         */
        cobs_enc_rslt = cobs_encode(cobs_bufs[cobs_wr_idx].bytes, 0x1fc, pb_encode_buf, stream.bytes_written);
        if (cobs_enc_rslt.status == COBS_ENCODE_OK) {
            cobs_bufs[cobs_wr_idx].bytes[cobs_enc_rslt.out_len] = 0x00;
            
            if (cyhal_uart_is_tx_active(&uart_obj) == false) {
                tx_len = cobs_enc_rslt.out_len+1;
                cyhal_uart_write_async(&uart_obj, cobs_bufs[cobs_wr_idx].bytes, tx_len);
            }
            else {
                cobs_buf_t* q_item = &cobs_bufs[cobs_wr_idx];
                cobs_bufs[cobs_wr_idx].len = cobs_enc_rslt.out_len+1;
                q_post = xQueueSendToBack(cobs_q, &q_item, (TickType_t)0);
                if (pdTRUE != q_post) {
                    CY_ASSERT(0);
                }
            }
            /* Increase the wr idx so that the next time we encode something it goes into the next buffer. */
            cobs_wr_idx += 1; cobs_wr_idx &= 3;
        }
        else {
            CY_ASSERT(0);
        }
    }
}
