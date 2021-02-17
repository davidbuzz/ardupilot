/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/buzz/ardupilot/modules/uavcan/dsdl/uavcan/equipment/esc/1034.Status.uavcan
 */

#ifndef __UAVCAN_EQUIPMENT_ESC_STATUS
#define __UAVCAN_EQUIPMENT_ESC_STATUS

#include <stdint.h>
#include "canard.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Generic ESC status.
# Unknown fields should be set to NAN.
#

uint32 error_count          # Resets when the motor restarts

float16 voltage             # Volt
float16 current             # Ampere. Can be negative in case of a regenerative braking.
float16 temperature         # Kelvin

int18 rpm                   # Negative value indicates reverse rotation

uint7 power_rating_pct      # Instant demand factor in percent (percent of maximum power); range 0% to 127%.

uint5 esc_index
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.esc.Status
saturated uint32 error_count
saturated float16 voltage
saturated float16 current
saturated float16 temperature
saturated int18 rpm
saturated uint7 power_rating_pct
saturated uint5 esc_index
******************************************************************************/

#define UAVCAN_EQUIPMENT_ESC_STATUS_ID                     1034
#define UAVCAN_EQUIPMENT_ESC_STATUS_NAME                   "uavcan.equipment.esc.Status"
#define UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE              (0xA9AF28AEA2FBB254ULL)

#define UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE               ((110 + 7)/8)

// Constants

typedef struct
{
    // FieldTypes
    uint32_t   error_count;                   // bit len 32
    float      voltage;                       // float16 Saturate
    float      current;                       // float16 Saturate
    float      temperature;                   // float16 Saturate
    int32_t    rpm;                           // bit len 18
    uint8_t    power_rating_pct;              // bit len 7
    uint8_t    esc_index;                     // bit len 5

} uavcan_equipment_esc_Status;

static inline
uint32_t uavcan_equipment_esc_Status_encode(uavcan_equipment_esc_Status* source, void* msg_buf);

static inline
int32_t uavcan_equipment_esc_Status_decode(const CanardRxTransfer* transfer, uint16_t payload_len, uavcan_equipment_esc_Status* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_equipment_esc_Status_encode_internal(uavcan_equipment_esc_Status* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_equipment_esc_Status_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, uavcan_equipment_esc_Status* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/buzz/ardupilot/modules/uavcan/dsdl/uavcan/equipment/esc/1034.Status.uavcan
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
  * @brief uavcan_equipment_esc_Status_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_equipment_esc_Status_encode_internal(uavcan_equipment_esc_Status* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->error_count); // 4294967295
    offset += 32;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->voltage);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->voltage;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->current);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->current;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->temperature);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->temperature;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;
    source->rpm = CANARD_INTERNAL_SATURATE(source->rpm, 131071)
    canardEncodeScalar(msg_buf, offset, 18, (void*)&source->rpm); // 131071
    offset += 18;

    source->power_rating_pct = CANARD_INTERNAL_SATURATE_UNSIGNED(source->power_rating_pct, 127)
    canardEncodeScalar(msg_buf, offset, 7, (void*)&source->power_rating_pct); // 127
    offset += 7;

    source->esc_index = CANARD_INTERNAL_SATURATE_UNSIGNED(source->esc_index, 31)
    canardEncodeScalar(msg_buf, offset, 5, (void*)&source->esc_index); // 31
    offset += 5;

    return offset;
}

/**
  * @brief uavcan_equipment_esc_Status_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_equipment_esc_Status_encode(uavcan_equipment_esc_Status* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_equipment_esc_Status_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_equipment_esc_Status_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_esc_Status dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_esc_Status_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_equipment_esc_Status* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 32, false, (void*)&dest->error_count);
    if (ret != 32)
    {
        goto uavcan_equipment_esc_Status_error_exit;
    }
    offset += 32;

    // float16 special handling
    ret = canardDecodeScalar(transfer, (uint32_t)offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_esc_Status_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->voltage = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->voltage = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, (uint32_t)offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_esc_Status_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->current = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->current = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, (uint32_t)offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_esc_Status_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->temperature = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->temperature = (float)tmp_float;
#endif
    offset += 16;

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 18, true, (void*)&dest->rpm);
    if (ret != 18)
    {
        goto uavcan_equipment_esc_Status_error_exit;
    }
    offset += 18;

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 7, false, (void*)&dest->power_rating_pct);
    if (ret != 7)
    {
        goto uavcan_equipment_esc_Status_error_exit;
    }
    offset += 7;

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 5, false, (void*)&dest->esc_index);
    if (ret != 5)
    {
        goto uavcan_equipment_esc_Status_error_exit;
    }
    offset += 5;
    return offset;

uavcan_equipment_esc_Status_error_exit:
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
  * @brief uavcan_equipment_esc_Status_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_esc_Status dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_esc_Status_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  uavcan_equipment_esc_Status* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_equipment_esc_Status); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_equipment_esc_Status_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_EQUIPMENT_ESC_STATUS