# add source and header files when compiling the given target.
# although it is not necessary to add header files, we also do that for easing the management of IDE.
target_sources(${CURRENT_EXE_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Sub.h
        ${CMAKE_CURRENT_LIST_DIR}/radio.cpp
        ${CMAKE_CURRENT_LIST_DIR}/commands_logic.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_surftrak.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RC_Channel.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_poshold.cpp
        ${CMAKE_CURRENT_LIST_DIR}/surface_bottom_detector.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Sub.h
        ${CMAKE_CURRENT_LIST_DIR}/Parameters.h
        ${CMAKE_CURRENT_LIST_DIR}/AP_Arming_Sub.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_acro.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RC_Channel.h
        ${CMAKE_CURRENT_LIST_DIR}/config.h
        ${CMAKE_CURRENT_LIST_DIR}/script_button.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_surface.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Mavlink.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_althold.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_guided.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Log.cpp
        ${CMAKE_CURRENT_LIST_DIR}/fence.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Parameters.cpp
        ${CMAKE_CURRENT_LIST_DIR}/terrain.cpp
        ${CMAKE_CURRENT_LIST_DIR}/version.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_circle.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Attitude.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_manual.cpp
        ${CMAKE_CURRENT_LIST_DIR}/UserVariables.h
        ${CMAKE_CURRENT_LIST_DIR}/mode.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Sub.cpp
        ${CMAKE_CURRENT_LIST_DIR}/defines.h
        ${CMAKE_CURRENT_LIST_DIR}/joystick.cpp
        ${CMAKE_CURRENT_LIST_DIR}/turn_counter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/motors.cpp
        ${CMAKE_CURRENT_LIST_DIR}/UserCode.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_motordetect.cpp
        ${CMAKE_CURRENT_LIST_DIR}/sensors.cpp
        ${CMAKE_CURRENT_LIST_DIR}/commands.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Mavlink.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_auto.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_stabilize.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ArduSub.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Sub.cpp
        ${CMAKE_CURRENT_LIST_DIR}/failsafe.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AP_State.cpp
        ${CMAKE_CURRENT_LIST_DIR}/script_button.cpp
        ${CMAKE_CURRENT_LIST_DIR}/system.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode.h
        ${CMAKE_CURRENT_LIST_DIR}/inertia.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AP_Arming_Sub.cpp
)

# add current directory to the compiler included directories when compiling the given target.
target_include_directories(${CURRENT_EXE_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

