#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * pose_frame.h  –  Binary frame definition
 *
 * Total: 20 bytes
 *
 *  Offset  Size  Field
 *  ──────  ────  ──────────────────────────────────────────────────────
 *   0      1     header   – magic byte 0xA5
 *   1      3     pos_x    – int24, signed, scale ÷10000 → metres (0.1mm res)
 *   4      3     pos_y    – int24, signed, scale ÷10000 → metres
 *   7      3     pos_z    – int24, signed, scale ÷10000 → metres
 *  10      2     quat_w   – int16, signed, scale ÷32767 → [-1, 1]
 *  12      2     quat_x   – int16, signed, scale ÷32767
 *  14      2     quat_y   – int16, signed, scale ÷32767
 *  16      2     quat_z   – int16, signed, scale ÷32767
 *  18      2     crc16    – CRC-16/CCITT-FALSE over bytes [0..17], little-endian
 *  ──────  ────
 *  20      total
 *
 * Encoding helpers
 * ─────────────────
 *  Position  :  raw = (int32_t)round(metres * 10000.0f)   range ±999,000 → fits int24
 *  Quaternion:  raw = (int16_t)round(component * 32767.0f) range ±1.0    → fits int16
 */

#define FRAME_HEADER      0xA5u
#define FRAME_SIZE        20u

#define POS_SCALE         10000.0f   /* raw = metres × 10000  (0.1 mm / LSB) */
#define QUAT_SCALE        32767.0f   /* raw = component × 32767              */

/* ── Packed struct (no padding) ─────────────────────────────────────────── */

#pragma pack(push, 1)
typedef struct {
    uint8_t  header;           /*  1 byte  – must be FRAME_HEADER            */
    uint8_t  pos_x[3];         /*  3 bytes – int24 little-endian             */
    uint8_t  pos_y[3];
    uint8_t  pos_z[3];
    int16_t  quat_w;           /*  2 bytes – int16 little-endian             */
    int16_t  quat_x;
    int16_t  quat_y;
    int16_t  quat_z;
    uint16_t crc16;            /*  2 bytes – CRC-16/CCITT-FALSE, LE          */
} pose_frame_t;                /* 20 bytes total                             */
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
 * @param  x,y,z  Position in metres  (range ±99.9 m, 0.1 mm resolution)
 * @param  w,qx,qy,qz  Unit quaternion components (range [-1, 1])
 */
static inline void pose_frame_build(pose_frame_t *f,
                                    float x,  float y,  float z,
                                    float w,  float qx, float qy, float qz)
{
    f->header = FRAME_HEADER;
    int24_encode(f->pos_x, (int32_t)(x  * POS_SCALE));
    int24_encode(f->pos_y, (int32_t)(y  * POS_SCALE));
    int24_encode(f->pos_z, (int32_t)(z  * POS_SCALE));
    f->quat_w = (int16_t)(w  * QUAT_SCALE);
    f->quat_x = (int16_t)(qx * QUAT_SCALE);
    f->quat_y = (int16_t)(qy * QUAT_SCALE);
    f->quat_z = (int16_t)(qz * QUAT_SCALE);
    f->crc16  = crc16_ccitt((const uint8_t *)f, FRAME_SIZE - 2);
}

/**
 * @brief  Validate header magic and CRC of a received frame.
 * @return 1 if valid, 0 if not.
 */
static inline int pose_frame_validate(const pose_frame_t *f)
{
    if (f->header != FRAME_HEADER) return 0;
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