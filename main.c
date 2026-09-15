/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the UART Ringbuffer Example for
*              ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <stdio.h>
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_peripherals.h"
#include "cy_scb_uart.h"
#include "cy_sysint.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Size of the software ring buffer (bytes) */
#define UART_RING_BUF_SIZE         (512u)

/* Number of bytes to batch-receive before echoing; set to half FIFO depth */
#define UART_RECEIVE_CHUNK         (8u)

/* NVIC mux index for the UART system interrupt (must not conflict with others) */
#define UART_CPU_IRQ               NvicMux2_IRQn

/* Shift value used to encode the CPU IRQ in cy_stc_sysint_t.intrSrc */
#define CPU_IRQ_NUMBER_SHIFT       (16u)

/* UART interrupt priority */
#define UART_IRQ_PRIORITY          (4u)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* PDL UART context (tracks ring buffer state, async transfers, etc.) */
static cy_stc_scb_uart_context_t g_uart_context;

/* HAL UART object required by retarget-io */
static mtb_hal_uart_t g_uart_hal_obj;

/* Software ring buffer backing memory */
static uint8_t g_uart_rx_ring[UART_RING_BUF_SIZE];

/* Local RX staging buffer used when draining the ring buffer */
static uint8_t g_uart_rx_data[UART_RECEIVE_CHUNK];

/* Flag set inside the callback; consumed by the main loop */
static volatile bool g_rx_done = false;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void uart_isr(void);
static void uart_event_callback(uint32_t events);

/*******************************************************************************
* Function Name: uart_isr
* Summary:
*  UART interrupt service routine.
*  Calls the PDL high-level handler which updates the ring buffer and fires
*  registered callbacks.
*******************************************************************************/
static void uart_isr(void)
{
    Cy_SCB_UART_Interrupt(UART_HW, &g_uart_context);
}

/*******************************************************************************
* Function Name: uart_event_callback
********************************************************************************
* Summary:
*  Called by Cy_SCB_UART_Interrupt() when a registered event occurs.
*  CY_SCB_UART_RECEIVE_DONE_EVENT: the requested number of bytes has been
*  moved from the ring buffer into g_uart_rx_data.  Echo them back and
*  re-arm the receive request for the next chunk.
*******************************************************************************/
static void uart_event_callback(uint32_t events)
{
    if (events & CY_SCB_UART_RECEIVE_DONE_EVENT)
    {
        /* Echo received chunk back to the host */
        Cy_SCB_UART_Transmit(UART_HW, g_uart_rx_data, UART_RECEIVE_CHUNK,
                             &g_uart_context);
        g_rx_done = true;
    }

    if (events & CY_SCB_UART_RECEIVE_ERR_EVENT)
    {
        /* Clear overrun / framing errors and continue */
        Cy_SCB_UART_ClearRxFifo(UART_HW);
    }

    if (events & CY_SCB_UART_RB_FULL_EVENT)
    {
        /* Ring buffer is full; drain a chunk so it does not overflow */
        Cy_SCB_UART_Receive(UART_HW, g_uart_rx_data, UART_RECEIVE_CHUNK,
                            &g_uart_context);
    }
}

/******************************************************************************
* Function Name: main
*******************************************************************************
* Summary:
*  1. Initialise the BSP.
*  2. Initialise UART using Device Configurator-generated structures.
*  3. Attach the 512-byte software ring buffer.
*  4. Register the event callback and arm the first async receive.
*  5. Configure the UART interrupt via Cy_SysInt_Init().
*  6. Redirect printf to the UART using retarget-io.
*  7. Loop: re-arm receive after each completed chunk.
*
* Parameters:
*  void
*
* Return:
*  int
*
******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* -------------------------------------------------------------------------
     * 1. BSP initialisation (clocks, power, pin mux from Device Configurator)
     * ---------------------------------------------------------------------- */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
        for (;;)
        {
        }
    }

    /* -------------------------------------------------------------------------
     * 2. UART PDL high-level initialisation
     *    UART_HW, UART_config come from cycfg_peripherals.h (Device Configurator)
     * ---------------------------------------------------------------------- */
    result = (cy_rslt_t)Cy_SCB_UART_Init(UART_HW, &UART_config, &g_uart_context);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
        for (;;)
        {
        }
    }

    /* -------------------------------------------------------------------------
     * 3. Attach the software ring buffer BEFORE enabling the peripheral so
     *    received bytes are stored immediately when Cy_SCB_UART_Enable() is called.
     * ---------------------------------------------------------------------- */
    Cy_SCB_UART_StartRingBuffer(UART_HW, g_uart_rx_ring, UART_RING_BUF_SIZE,
                                &g_uart_context);

    /* -------------------------------------------------------------------------
     * 4. Register the event callback
     * ---------------------------------------------------------------------- */
    Cy_SCB_UART_RegisterCallback(UART_HW, uart_event_callback, &g_uart_context);

    /* -------------------------------------------------------------------------
     * 5. Enable the UART hardware
     * ---------------------------------------------------------------------- */
    Cy_SCB_UART_Enable(UART_HW);

    /* -------------------------------------------------------------------------
     * 6. Configure the UART interrupt
     *    intrSrc encodes: [31:16] CPU IRQ (NvicMuxN), [15:0] system IRQ source
     * ---------------------------------------------------------------------- */
    cy_stc_sysint_t uart_irq_cfg = {
        .intrSrc      = ((UART_CPU_IRQ << CPU_IRQ_NUMBER_SHIFT) | UART_IRQ),
        .intrPriority = UART_IRQ_PRIORITY,
    };
    result = (cy_rslt_t)Cy_SysInt_Init(&uart_irq_cfg, uart_isr);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
        for (;;)
        {
        }
    }
    NVIC_EnableIRQ((IRQn_Type)UART_CPU_IRQ);

    /* -------------------------------------------------------------------------
     * 7. Setup the HAL UART object for retarget-io (printf)
     *    UART_hal_config is also generated by the Device Configurator.
     * ---------------------------------------------------------------------- */
    result = mtb_hal_uart_setup(&g_uart_hal_obj, &UART_hal_config, &g_uart_context,
                                NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
        for (;;)
        {
        }
    }

    result = cy_retarget_io_init(&g_uart_hal_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
        for (;;)
        {
        }
    }

    /* Enable global interrupts after all peripherals are ready */
    __enable_irq();

    /* -------------------------------------------------------------------------
     * 8. Arm the first asynchronous receive
     *    When UART_RECEIVE_CHUNK bytes arrive in the ring buffer,
     *    CY_SCB_UART_RECEIVE_DONE_EVENT fires and uart_event_callback() echoes
     *    them back and sets g_rx_done.
     * ---------------------------------------------------------------------- */
    result = (cy_rslt_t)Cy_SCB_UART_Receive(UART_HW, g_uart_rx_data,
                                             UART_RECEIVE_CHUNK, &g_uart_context);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(false);
        for (;;)
        {
        }
    }

    printf("\r\n*** UART Ring Buffer Demonstration ***\r\n");
    printf("Send data at 115200-8N1. Each %u bytes will be echoed back.\r\n\r\n",
           (unsigned)UART_RECEIVE_CHUNK);
    printf("Please enter characters of 8 bytes\r\n\r\n");

    /* -------------------------------------------------------------------------
     * 9. Main loop
     * ---------------------------------------------------------------------- */
    for (;;)
    {
        if (g_rx_done)
        {
            g_rx_done = false;

            /* Re-arm for the next chunk once transmit has finished */
            while (!Cy_SCB_UART_IsTxComplete(UART_HW))
            {
                /* wait */
            }

            result = (cy_rslt_t)Cy_SCB_UART_Receive(UART_HW, g_uart_rx_data,
                                                     UART_RECEIVE_CHUNK,
                                                     &g_uart_context);
            printf("\r\n");
            if (result != CY_RSLT_SUCCESS)
            {
                CY_ASSERT(false);
                for (;;)
                {
                }
            }
        }
    }
}

/* [] END OF FILE */
