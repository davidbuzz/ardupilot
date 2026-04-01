##############################################################################
# Laurel reuses the RP2350 board build plumbing from Pico2. This stays local so
# the board can diverge later without disturbing the original Pico2 target.
##############################################################################

include $(AP_HAL)/hwdef/Pico2/chibios_board.mk

# Use the common RP2350 core1 dispatcher from the Pico2 target.
CSRC := $(filter-out $(AP_HAL)/hwdef/Pico2/c1_main.c,$(CSRC))
CSRC += $(AP_HAL)/hwdef/Pico2/c1_main.c