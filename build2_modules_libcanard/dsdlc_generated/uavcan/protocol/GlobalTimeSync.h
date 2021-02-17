/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/buzz/ardupilot/modules/uavcan/dsdl/uavcan/protocol/4.GlobalTimeSync.uavcan
 */

#ifndef __UAVCAN_PROTOCOL_GLOBALTIMESYNC
#define __UAVCAN_PROTOCOL_GLOBALTIMESYNC

#include <stdint.h>
#include "canard.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Global time synchronization.
# Any node that publishes timestamped data must use this time reference.
#
# Please refer to the specification to learn about the synchronization algorithm.
#

#
# Broadcasting period must be within this range.
#
uint16 MAX_BROADCASTING_PERIOD_MS = 1100            # Milliseconds
uint16 MIN_BROADCASTING_PERIOD_MS = 40              # Milliseconds

#
# Synchronization slaves may switch to a new source if the current master was silent for this amount of time.
#
uint16 RECOMMENDED_BROADCASTER_TIMEOUT_MS = 2200    # Milliseconds

#
# Time in microseconds when the PREVIOUS GlobalTimeSync message was transmitted.
# If this message is the first one, this field must be zero.
#
truncated uint56 previous_transmission_timestamp_usec # Microseconds
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.GlobalTimeSync
truncated uint56 previous_transmission_timestamp_usec
******************************************************************************/

#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID                  4
#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_NAME                "uavcan.protocol.GlobalTimeSync"
#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_SIGNATURE           (0x20271116A793C2DBULL)

#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_MAX_SIZE            ((56 + 7)/8)

// Constants
#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_MAX_BROADCASTING_PERIOD_MS          1100 // 1100
#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_MIN_BROADCASTING_PERIOD_MS            40 // 40
#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_RECOMMENDED_BROADCASTER_TIMEOUT_MS       2200 // 2200

typedef struct
{
    // FieldTypes
    uint64_t   previous_transmission_timestamp_usec; // bit len 56

} uavcan_protocol_GlobalTimeSync;

static inline
uint32_t uavcan_protocol_GlobalTimeSync_encode(uavcan_protocol_GlobalTimeSync* source, void* msg_buf);

static inline
int32_t uavcan_protocol_GlobalTimeSync_decode(const CanardRxTransfer* transfer, uint16_t payload_len, uavcan_protocol_GlobalTimeSync* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_GlobalTimeSync_encode_internal(uavcan_protocol_GlobalTimeSync* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_GlobalTimeSync_decode_internal(const CanardRxTransfer* transfer, uint16_t payload_len, uavcan_protocol_GlobalTimeSync* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/buzz/ardupilot/modules/uavcan/dsdl/uavcan/protocol/4.GlobalTimeSync.uavcan
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
  * @brief uavcan_protocol_GlobalTimeSync_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_GlobalTimeSync_encode_internal(uavcan_protocol_GlobalTimeSync* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    canardEncodeScalar(msg_buf, offset, 56, (void*)&source->previous_transmission_timestamp_usec); // 72057594037927935
    offset += 56;

    return offset;
}

/**
  * @brief uavcan_protocol_GlobalTimeSync_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_GlobalTimeSync_encode(uavcan_protocol_GlobalTimeSync* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_GlobalTimeSync_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_GlobalTimeSync_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_GlobalTimeSync dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_GlobalTimeSync_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_GlobalTimeSync* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 56, false, (void*)&dest->previous_transmission_timestamp_usec);
    if (ret != 56)
    {
        goto uavcan_protocol_GlobalTimeSync_error_exit;
    }
    offset += 56;
    return offset;

uavcan_protocol_GlobalTimeSync_error_exit:
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
  * @brief uavcan_protocol_GlobalTimeSync_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_GlobalTimeSync dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_GlobalTimeSync_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_GlobalTimeSync* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_GlobalTimeSync); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_GlobalTimeSync_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_PROTOCOL_GLOBALTIMESYNC