// Shared header describing the RTOS payload that the packaging tool
// (tools/pack_migrator.py) pre-positions at flash 0x110000. The 533 KB RTOS
// full-flash image itself starts at 0x111000 (one sector after this header).
//
// Integrity is a standard zlib/IEEE CRC-32 (poly 0xEDB88320, init/xorout
// 0xFFFFFFFF) rather than SHA-256: the payload is not adversarial, so we only
// need to catch transfer/flash corruption, and CRC-32 does that with a tiny,
// dependency-free implementation identical on host (zlib.crc32) and device.
#pragma once
#include <stdint.h>

#define MIG_MAGIC   0x47494D4Bu   // 'KMIG' little-endian
#define MIG_VERSION 1u

#define MIG_HDR_ADDR      0x110000u   // this header (one 4 KB sector)
#define MIG_PAYLOAD_ADDR  0x111000u   // RTOS full-flash image starts here

typedef struct __attribute__((packed)) {
  uint32_t magic;         // MIG_MAGIC
  uint32_t version;       // MIG_VERSION
  uint32_t target_base;   // flash dest of payload byte 0 (== 0)
  uint32_t length;        // payload length in bytes (whole sectors; == 0x83000)
  uint32_t sector_count;  // length / 4096 (== 131)
  uint32_t payload_crc32; // zlib CRC-32 over [MIG_PAYLOAD_ADDR, +length)
  uint32_t hdr_crc32;     // zlib CRC-32 over the 24 bytes before this field
} payload_hdr_t;

#define MIG_HDR_CRC_LEN 24u  // offsetof(payload_hdr_t, hdr_crc32)
