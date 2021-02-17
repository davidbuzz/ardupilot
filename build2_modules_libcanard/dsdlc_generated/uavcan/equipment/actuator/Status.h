/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/buzz/ardupilot/modules/uavcan/dsdl/uavcan/equipment/actuator/1011.Status.uavcan
 */

#ifndef __UAVCAN_EQUIPMENT_ACTUATOR_STATUS
#define __UAVCAN_EQUIPMENT_ACTUATOR_STATUS

#include <stdint.h>
#include "canard.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Generic actuator feedback, if available.
# Unknown fields should be set to NAN.
#

uint8 actuator_id

#
# Whether the units are linear or angular depends on the actuator type (refer to the Command data type).
#
float16 position        # meter or radian
float16 force           # Newton or Newton metre
float16 speed           # meter per second or radian per second

void1
uint7 POWER_RATING_PCT_UNKNOWN = 127
uint7 power_rating_pct                # 0 - unloaded, 100 - full load
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.actuator.Status
saturated uint8 actuator_id
saturated float16 position
saturated float16 force
saturated float16 speed
void1
saturated uint7 power_rating_pct
******************************************************************************/

#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID                1011
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_NAME              "uavcan.equipment.actuator.Status"
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE         (0x5E9BBA44FAF1EA04ULL)

#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE          ((64 + 7)/8)

// Constants
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN           127 // 127

typedef struct
{
    // FieldTypes
    uint8_t    actuator_id;                   // bit len 8
    float      position;                      // float16 Saturate
    float      force;                         // float16 Saturate
    float      speed;                         // float16 Saturate
    // void1
    uint8_t    power_rating_pct;              // bit len 7

} uavcan_equipment_actuator_Status;

static inline
uint32_t uavcan_equipment_actuator_Status_encode(uavcan_equipment_actuator_Status* source, void* msg_buf);

static inline
int32_t uavcan_equipment_actuator_Status_decode(const CanardRxTransfer* transfer, uint16_t payload_len, uavcan_equipment_actuator_Status* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_equipment_actuator_Status_encode_internal(uavcan_equipment_actuator_Status* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_equipment_actuator_Status_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, uavcan_equipment_actuator_Status* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/buzz/ardupilot/modules/uavcan/dsdl/uavcan/equipment/actuator/1011.Status.uavcan
 */

#ifndef CANARD_INTERNAL_SATURATE
#define CANARD_INTERNAL_SATURATE(x, max) ( ((x) > max) ? max : ( (-(x) > max) ? (-max) : (x) ) );
#endif

#ifndef CANARD_INTERNAL_SATURATE_UNSIGNED
#define CANARD_INTERNAL_SATURATE_UNSIGNED(x, max) ( ((x) >= max) ? max : (x) );
#endif

#if defined(__GNUC__)
# define CANARD_MAYBE_UNUSED(x) x __attribute__((unused))
#else
# define CANARD_MAYBE_UNUSED(x) x
#endif

/**
  * @brief uavcan_equipment_actuator_Status_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_equipment_actuator_Status_encode_internal(uavcan_equipment_actuator_Status* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->actuator_id); // 255
    offset += 8;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->position);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->position;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->force);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->force;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->speed);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->speed;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // Void1
    offset += 1;
    source->power_rating_pct = CANARD_INTERNAL_SATURATE_UNSIGNED(source->power_rating_pct, 127)
    canardEncodeScalar(msg_buf, offset, 7, (void*)&source->power_rating_pct); // 127
    offset += 7;

    return offset;
}

/**
  * @brief uavcan_equipment_actuator_Status_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_equipment_actuator_Status_encode(uavcan_equipment_actuator_Status* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_equipment_actuator_Status_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_equipment_actuator_Status_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_actuator_Status dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_actuator_Status_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_equipment_actuator_Status* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 8, false, (void*)&dest->actuator_id);
    if (ret != 8)
    {
        goto uavcan_equipment_actuator_Status_error_exit;
    }
    offset += 8;

    // float16 special handling
    ret = canardDecodeScalar(transfer, (uint32_t)offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_actuator_Status_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->position = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->position = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, (uint32_t)offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_actuator_Status_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->force = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->force = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, (uint32_t)offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_actuator_Status_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->speed = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->speed = (float)tmp_float;
#endif
    offset += 16;

    // Void1
    offset += 1;

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 7, false, (void*)&dest->power_rating_pct);
    if (ret != 7)
    {
        goto uavcan_equipment_actuator_Status_error_exit;
    }
    offset += 7;
    return offset;

uavcan_equipment_actuator_Status_error_exit:
    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return -CANARD_ERROR_INTERNAL;
    }
}

/**
  * @brief uavcan_equipment_actuator_Status_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_actuator_Status dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_actuator_Status_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  uavcan_equipment_actuator_Status* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_equipment_actuator_Status); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_equipment_actuator_Status_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_EQUIPMENT_ACTUATOR_STATUS