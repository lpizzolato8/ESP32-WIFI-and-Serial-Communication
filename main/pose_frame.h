#pragma once
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * pose_frame.h  –  Binary frame definition (must match pose_frame.py)
 *
 * Total: 21 bytes, all fields little-endian
 *
 *  Offset  Size  Field
 *  ──────  ────  ──────────────────────────────────────────────────────
 *   0      2     magic    – uint16 0xABCD (bytes CD AB on the wire)
 *   2      3     pos_x    – int24, signed, scale ÷10000 → metres (0.1mm res)
 *   5      3     pos_y    – int24, signed, scale ÷10000 → metres
 *   8      3     pos_z    – int24, signed, scale ÷10000 → metres
 *  11      2     quat_w   – int16, signed, scale ÷32767 → [-1, 1]
 *  13      2     quat_x   – int16, signed, scale ÷32767
 *  15      2     quat_y   – int16, signed, scale ÷32767
 *  17      2     quat_z   – int16, signed, scale ÷32767
 *  19      2     crc16    – CRC-16/CCITT-FALSE over bytes [0..18]
 *  ──────  ────
 *  21      total
 *
 * Encoding helpers
 * ─────────────────
 *  Position  :  raw = (int32_t)lroundf(metres * 10000.0f)    range ±838.86 m
 *  Quaternion:  raw = (int16_t)lroundf(component * 32767.0f) range ±1.0
 *
 * The struct relies on a little-endian target (ESP32, STM32).
 */

#define FRAME_MAGIC       0xABCDu
#define FRAME_SIZE        21u

#define POS_SCALE         10000.0f   /* raw = metres × 10000  (0.1 mm / LSB) */
#define QUAT_SCALE        32767.0f   /* raw = component × 32767              */

/* ── Packed struct (no padding) ─────────────────────────────────────────── */

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;            /*  2 bytes – must be FRAME_MAGIC             */
    uint8_t  pos_x[3];         /*  3 bytes – int24 little-endian             */
    uint8_t  pos_y[3];
    uint8_t  pos_z[3];
    int16_t  quat_w;           /*  2 bytes – int16 little-endian             */
    int16_t  quat_x;
    int16_t  quat_y;
    int16_t  quat_z;
    uint16_t crc16;            /*  2 bytes – CRC-16/CCITT-FALSE, LE          */
} pose_frame_t;                /* 21 bytes total                             */

#ifdef __cplusplus
static_assert(sizeof(pose_frame_t) == FRAME_SIZE, "pose_frame_t must be 21 bytes");
#else
_Static_assert(sizeof(pose_frame_t) == FRAME_SIZE, "pose_frame_t must be 21 bytes");
#endif
#pragma pack(pop)

/* ── int24 helpers ──────────────────────────────────────────────────────── */

/** Encode a signed 32-bit value into a 3-byte little-endian buffer. */
static inline void int24_encode(uint8_t dst[3], int32_t v)
{
    dst[0] = (uint8_t)( v        & 0xFF);
    dst[1] = (uint8_t)((v >>  8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
}

/** Decode a 3-byte little-endian buffer into a signed 32-bit value. */
static inline int32_t int24_decode(const uint8_t src[3])
{
    int32_t v = (int32_t)src[0]
              | ((int32_t)src[1] << 8)
              | ((int32_t)src[2] << 16);
    /* Sign-extend from bit 23 */
    if (v & 0x800000) v |= (int32_t)0xFF000000;
    return v;
}

/* ── CRC-16/CCITT-FALSE ─────────────────────────────────────────────────── */

/** Compute CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect). */
static inline uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ── Build / parse helpers ──────────────────────────────────────────────── */

/**
 * @brief  Fill a pose_frame_t from float inputs and append CRC.
 *
 * @param  f      Output frame (must point to valid memory)
 * @param  x,y,z  Position in metres  (range ±838.86 m, 0.1 mm resolution)
 * @param  w,qx,qy,qz  Unit quaternion components (range [-1, 1])
 */
static inline void pose_frame_build(pose_frame_t *f,
                                    float x,  float y,  float z,
                                    float w,  float qx, float qy, float qz)
{
    f->magic  = FRAME_MAGIC;
    /* Round (not truncate) to match pose_frame.py. */
    int24_encode(f->pos_x, (int32_t)lroundf(x  * POS_SCALE));
    int24_encode(f->pos_y, (int32_t)lroundf(y  * POS_SCALE));
    int24_encode(f->pos_z, (int32_t)lroundf(z  * POS_SCALE));
    f->quat_w = (int16_t)lroundf(w  * QUAT_SCALE);
    f->quat_x = (int16_t)lroundf(qx * QUAT_SCALE);
    f->quat_y = (int16_t)lroundf(qy * QUAT_SCALE);
    f->quat_z = (int16_t)lroundf(qz * QUAT_SCALE);
    f->crc16  = crc16_ccitt((const uint8_t *)f, FRAME_SIZE - 2);
}

/**
 * @brief  Validate magic and CRC of a received frame.
 * @return 1 if valid, 0 if not.
 */
static inline int pose_frame_validate(const pose_frame_t *f)
{
    if (f->magic != FRAME_MAGIC) return 0;
    uint16_t expected = crc16_ccitt((const uint8_t *)f, FRAME_SIZE - 2);
    return (f->crc16 == expected) ? 1 : 0;
}

/**
 * @brief  Decode position from a validated frame (metres).
 */
static inline void pose_frame_get_pos(const pose_frame_t *f,
                                      float *x, float *y, float *z)
{
    *x = (float)int24_decode(f->pos_x) / POS_SCALE;
    *y = (float)int24_decode(f->pos_y) / POS_SCALE;
    *z = (float)int24_decode(f->pos_z) / POS_SCALE;
}

/**
 * @brief  Decode quaternion from a validated frame (normalised components).
 */
static inline void pose_frame_get_quat(const pose_frame_t *f,
                                       float *w, float *qx, float *qy, float *qz)
{
    *w  = (float)f->quat_w / QUAT_SCALE;
    *qx = (float)f->quat_x / QUAT_SCALE;
    *qy = (float)f->quat_y / QUAT_SCALE;
    *qz = (float)f->quat_z / QUAT_SCALE;
}

#ifdef __cplusplus
}
#endif