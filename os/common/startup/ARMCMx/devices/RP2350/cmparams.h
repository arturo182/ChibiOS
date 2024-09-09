/*
    ChibiOS - Copyright (C) 2006..2021 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    RP2040/cmparams.h
 * @brief   ARM Cortex-M33 parameters for the RP2350.
 *
 * @defgroup ARMCMx_RP2350 RP2350 Specific Parameters
 * @ingroup ARMCMx_SPECIFIC
 * @details This file contains the Cortex-M33 specific parameters for the
 *          RP2350 platform.
 * @{
 */

#ifndef CMPARAMS_H
#define CMPARAMS_H

/**
 * @brief   Cortex core model.
 */
#define CORTEX_MODEL            33U

/**
 * @brief   Floating Point unit presence.
 */
#define CORTEX_HAS_FPU          1

/**
 * @brief   Number of bits in priority masks.
 */
#define CORTEX_PRIORITY_BITS    4

/* If the device type is not externally defined, for example from the Makefile,
   then a file named board.h is included. This file must contain a device
   definition compatible with the vendor include file.*/
#if !defined (RP2350)
#include "board.h"
#endif

/**
 * @brief   Number of interrupt vectors.
 * @note    This number does not include the 16 system vectors and must be
 *          rounded to a multiple of 8.
 */
#define CORTEX_NUM_VECTORS      56

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/* Including the device CMSIS header. Note, we are not using the definitions
   from this header because we need this file to be usable also from
   assembler source files. We verify that the info matches instead.*/
#include "hardware/regs/accessctrl.h"
#include "hardware/regs/adc.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/bootram.h"
#include "hardware/regs/busctrl.h"
#include "hardware/regs/clocks.h"
#include "hardware/regs/coresight_trace.h"
#include "hardware/regs/dma.h"
#include "hardware/regs/dreq.h"
#include "hardware/regs/glitch_detector.h"
#include "hardware/regs/hstx_ctrl.h"
#include "hardware/regs/hstx_fifo.h"
#include "hardware/regs/i2c.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/io_qspi.h"
#include "hardware/regs/m33.h"
#include "hardware/regs/m33_eppb.h"
#include "hardware/regs/otp.h"
#include "hardware/regs/otp_data.h"
#include "hardware/regs/pads_bank0.h"
#include "hardware/regs/pads_qspi.h"
#include "hardware/regs/pio.h"
#include "hardware/regs/pll.h"
#include "hardware/regs/powman.h"
#include "hardware/regs/psm.h"
#include "hardware/regs/pwm.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/resets.h"
#include "hardware/regs/riscv_dm.h"
#include "hardware/regs/rosc.h"
#include "hardware/regs/rp_ap.h"
#include "hardware/regs/rvcsr.h"
#include "hardware/regs/sha256.h"
#include "hardware/regs/sio.h"
#include "hardware/regs/spi.h"
#include "hardware/regs/syscfg.h"
#include "hardware/regs/sysinfo.h"
#include "hardware/regs/tbman.h"
#include "hardware/regs/ticks.h"
#include "hardware/regs/timer.h"
#include "hardware/regs/trng.h"
#include "hardware/regs/uart.h"
#include "hardware/regs/usb.h"
#include "hardware/regs/usb_device_dpram.h"
#include "hardware/regs/watchdog.h"
#include "hardware/regs/xip.h"
#include "hardware/regs/xip_aux.h"
#include "hardware/regs/xosc.h"
#include "hardware/structs/accessctrl.h"
#include "hardware/structs/adc.h"
#include "hardware/structs/bootram.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/busctrl.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/coresight_trace.h"
#include "hardware/structs/dma.h"
#include "hardware/structs/dma_debug.h"
#include "hardware/structs/glitch_detector.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/i2c.h"
#include "hardware/structs/interp.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/io_qspi.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/m33.h"
#include "hardware/structs/m33_eppb.h"
#include "hardware/structs/mpu.h"
#include "hardware/structs/nvic.h"
#include "hardware/structs/otp.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/structs/pads_qspi.h"
#include "hardware/structs/padsbank0.h"
#include "hardware/structs/pio.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/powman.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/sau.h"
#include "hardware/structs/scb.h"
#include "hardware/structs/sha256.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/spi.h"
#include "hardware/structs/syscfg.h"
#include "hardware/structs/sysinfo.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/tbman.h"
#include "hardware/structs/ticks.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/tmds_encode.h"
#include "hardware/structs/trng.h"
#include "hardware/structs/uart.h"
#include "hardware/structs/usb.h"
#include "hardware/structs/usb_dpram.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/xip.h"
#include "hardware/structs/xip_aux.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/structs/xosc.h"
#include "RP2350.h"

/*lint -save -e9029 [10.4] Signedness comes from external files, it is
  unpredictable but gives no problems.*/
#if CORTEX_MODEL != __CORTEX_M
#error "CMSIS __CORTEX_M mismatch"
#endif

#if CORTEX_PRIORITY_BITS != __NVIC_PRIO_BITS
#error "CMSIS __NVIC_PRIO_BITS mismatch"
#endif
/*lint -restore*/

#endif /* !defined(_FROM_ASM_) */

#endif /* CMPARAMS_H */

/** @} */
