#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <SRV_Channel/SRV_Channel_config.h>

#ifndef AP_MOTORS_TRI_ENABLED
#define AP_MOTORS_TRI_ENABLED 1
#endif  // AP_MOTORS_TRI_ENABLED

// Per-motor MOT_SPIN_MINx override (experimental, bench use).  When disabled the
// single MOT_SPIN_MIN is used for every motor, which is the stock behaviour and
// leaves this shared library byte-identical.  Enable per-board via hwdef.
#ifndef AP_MOTORS_SPIN_MIN_PER_MOTOR_ENABLED
#define AP_MOTORS_SPIN_MIN_PER_MOTOR_ENABLED 0
#endif

// Per-motor MOT_SPIN_ARMx armed-idle override (experimental, bench use).  When
// disabled the single MOT_SPIN_ARM applies to every motor (stock behaviour,
// byte-identical).  Enable per-board via hwdef.
#ifndef AP_MOTORS_SPIN_ARM_PER_MOTOR_ENABLED
#define AP_MOTORS_SPIN_ARM_PER_MOTOR_ENABLED 0
#endif

#ifndef AP_MOTORS_FRAME_DEFAULT_ENABLED
#define AP_MOTORS_FRAME_DEFAULT_ENABLED 1
#endif

#ifndef AP_MOTORS_FRAME_QUAD_ENABLED
#define AP_MOTORS_FRAME_QUAD_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_HEXA_ENABLED
#define AP_MOTORS_FRAME_HEXA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_OCTA_ENABLED
#define AP_MOTORS_FRAME_OCTA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_DECA_ENABLED
#define AP_MOTORS_FRAME_DECA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#define AP_MOTORS_FRAME_DODECAHEXA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_Y6_ENABLED
#define AP_MOTORS_FRAME_Y6_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
#ifndef AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#define AP_MOTORS_FRAME_OCTAQUAD_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
