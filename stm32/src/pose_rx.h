#pragma once
/**
 * pose_rx.h  –  Pose frame parser, failsafe and status frame builder.
 *
 * No HAL dependencies, so it also builds on the laptop for tests
 * (stm32/test/test_pose_rx.py).
 */
#include <stddef.h>
#include <stdint.h>
#include "pose_frame.h"

/* Frames sent to the ESP32 (relayed to the laptop; must match test_bridge.py
 * and main/udp_uart_bridge.c). Both are short so one can't hold the UART TX
 * line long enough to delay the other noticeably (67 B = 0.7 ms at 921600):
 *   status: "STM\xAA" | 4 counter | 57 pose_rx_status_t | 2 CRC-16 = 67 B
 *   echo  : "ECH\xAA" | 21 pose frame                    | 2 CRC-16 = 27 B  */

#define FAILSAFE_TIMEOUT_MS 100u   /* no valid pose for this long → failsafe */

/* Status at the start of the reverse payload; the rest is zero.
 * Python: struct "<4s6IB3xI21s" (57 bytes).                               */
#pragma pack(push, 1)
typedef struct {
    char         tag[4];          /* "STM1"                                 */
    uint32_t     uptime_ms;
    uint32_t     frames_ok;       /* valid pose frames received             */
    uint32_t     crc_fail;        /* frames with good magic but bad CRC     */
    uint32_t     bytes_skipped;   /* bytes discarded while re-syncing       */
    uint32_t     uart_errors;     /* overrun / framing / noise errors       */
    uint32_t     failsafe_count;  /* times failsafe was entered             */
    uint8_t      failsafe;        /* 1 = no valid pose within the timeout   */
    uint8_t      reserved[3];
    uint32_t     last_rx_age_ms;  /* ms since last valid pose, ~0 if none   */
    pose_frame_t last_pose;       /* raw copy of the last valid frame       */
} pose_rx_status_t;
#pragma pack(pop)
_Static_assert(sizeof(pose_rx_status_t) == 57, "status must be 57 bytes");

#define STATUS_FRAME_SIZE (4u + 4u + sizeof(pose_rx_status_t) + 2u)   /* 67 */
#define ECHO_FRAME_SIZE   (4u + FRAME_SIZE + 2u)                      /* 27 */

/** Feed one received UART byte. */
void pose_rx_feed(uint8_t b, uint32_t now_ms);

/** Drop a partly assembled frame (after a UART error or DMA restart). */
void pose_rx_reset_parser(void);

void pose_rx_count_uart_error(void);

/** Update the failsafe flag; call often. Returns 1 while in failsafe. */
int pose_rx_tick(uint32_t now_ms);

/** Write a STATUS_FRAME_SIZE status frame into out. */
void pose_rx_build_status(uint8_t out[STATUS_FRAME_SIZE], uint32_t counter,
                          uint32_t now_ms);

/** If a valid pose arrived since the last call, write an ECHO_FRAME_SIZE
 *  echo of the latest one into out and return 1; otherwise return 0. The
 *  laptop times this to measure the full round trip through the STM32. */
int pose_rx_take_echo(uint8_t out[ECHO_FRAME_SIZE]);

const pose_rx_status_t *pose_rx_status(void);
