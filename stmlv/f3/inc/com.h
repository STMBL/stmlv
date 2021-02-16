#include "stdint.h"

typedef struct {
  uint32_t crc : 8;
  uint32_t reply_id : 8;
  uint32_t en : 8;
  uint32_t df : 8;
  float torque_cmd[4]; 
} mcom_t;

typedef struct {
  uint32_t crc : 8;
  uint32_t reply_id : 8;
  uint32_t state : 8;
  uint32_t df : 8;
  float vel_fb;
} scom_t;

uint8_t crc8 (uint8_t * ptr, uint32_t len);