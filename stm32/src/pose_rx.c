#include <string.h>
#include "pose_rx.h"

static uint8_t  acc[FRAME_SIZE];   /* frame being assembled */
static uint32_t acc_len;

static pose_rx_status_t st;
static uint8_t  have_pose;
static uint32_t last_pose_ms;

static void on_pose(const pose_frame_t *f, uint32_t now_ms)
{
    /* A flight controller would feed this into its position estimator. */
    memcpy(&st.last_pose, f, sizeof *f);
    st.frames_ok++;
    have_pose    = 1;
    last_pose_ms = now_ms;
}

static void frame_complete(uint32_t now_ms)
{
    pose_frame_t f;
    memcpy(&f, acc, FRAME_SIZE);
    acc_len = 0;

    if (pose_frame_validate(&f)) {
        on_pose(&f, now_ms);
        return;
    }

    /* Bad CRC: drop the first magic byte and re-scan the rest, in case a
     * real frame started inside this one. Refeeding 20 bytes can't fill
     * another frame, so this recurses at most once.                       */
    st.crc_fail++;
    st.bytes_skipped++;
    uint8_t rest[FRAME_SIZE - 1];
    memcpy(rest, acc + 1, sizeof rest);
    for (uint32_t i = 0; i < sizeof rest; i++)
        pose_rx_feed(rest[i], now_ms);
}

void pose_rx_feed(uint8_t b, uint32_t now_ms)
{
    const uint8_t m0 = FRAME_MAGIC & 0xFF, m1 = FRAME_MAGIC >> 8;  /* CD AB */

    if (acc_len == 0) {
        if (b == m0) acc[acc_len++] = b;
        else         st.bytes_skipped++;
    } else if (acc_len == 1) {
        if (b == m1)      acc[acc_len++] = b;
        else if (b != m0) { acc_len = 0; st.bytes_skipped += 2; }
        else              st.bytes_skipped++;     /* CD CD: keep the second */
    } else {
        acc[acc_len++] = b;
        if (acc_len == FRAME_SIZE) frame_complete(now_ms);
    }
}

void pose_rx_reset_parser(void)
{
    acc_len = 0;
}

void pose_rx_count_uart_error(void)
{
    st.uart_errors++;
}

int pose_rx_tick(uint32_t now_ms)
{
    uint8_t fs = !have_pose || (now_ms - last_pose_ms) > FAILSAFE_TIMEOUT_MS;
    if (fs && !st.failsafe && have_pose) st.failsafe_count++;
    st.failsafe = fs;
    return fs;
}

void pose_rx_build_status(uint8_t out[REV_FRAME_SIZE], uint32_t counter,
                          uint32_t now_ms)
{
    memcpy(st.tag, "STM1", 4);
    st.uptime_ms      = now_ms;
    st.last_rx_age_ms = have_pose ? now_ms - last_pose_ms : 0xFFFFFFFFu;

    uint8_t *p = out;
    memcpy(p, "REV\xAA", 4);        p += 4;
    memcpy(p, &counter, 4);         p += 4;
    memset(p, 0, REV_PAYLOAD_SIZE);
    memcpy(p, &st, sizeof st);      p += REV_PAYLOAD_SIZE;
    uint16_t crc = crc16_ccitt(out, (uint16_t)(p - out));
    memcpy(p, &crc, 2);
}

const pose_rx_status_t *pose_rx_status(void)
{
    return &st;
}
