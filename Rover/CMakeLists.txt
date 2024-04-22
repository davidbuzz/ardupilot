# add source and header files when compiling the given target.
# although it is not necessary to add header files, we also do that for easing the management of IDE.
target_sources(${CURRENT_EXE_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/radio.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_loiter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RC_Channel.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Parameters.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_acro.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RC_Channel.h
        ${CMAKE_CURRENT_LIST_DIR}/config.h
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Mavlink.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_simple.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_guided.cpp
        ${CMAKE_CURRENT_LIST_DIR}/precision_landing.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Log.cpp
        ${CMAKE_CURRENT_LIST_DIR}/afs_rover.h
        ${CMAKE_CURRENT_LIST_DIR}/fence.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_follow.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Parameters.cpp
        ${CMAKE_CURRENT_LIST_DIR}/crash_check.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AP_Arming.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Rover.cpp
        ${CMAKE_CURRENT_LIST_DIR}/afs_rover.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AP_ExternalControl_Rover.h
        ${CMAKE_CURRENT_LIST_DIR}/version.h
        ${CMAKE_CURRENT_LIST_DIR}/AP_Rally.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Rover.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_circle.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AP_Arming.h
        ${CMAKE_CURRENT_LIST_DIR}/AP_ExternalControl_Rover.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_manual.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_smart_rtl.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_dock.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode.cpp
        ${CMAKE_CURRENT_LIST_DIR}/sailboat.cpp
        ${CMAKE_CURRENT_LIST_DIR}/defines.h
        ${CMAKE_CURRENT_LIST_DIR}/sailboat.h
        ${CMAKE_CURRENT_LIST_DIR}/sensors.cpp
        ${CMAKE_CURRENT_LIST_DIR}/commands.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Mavlink.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_auto.cpp
        ${CMAKE_CURRENT_LIST_DIR}/motor_test.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ekf_check.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Rover.h
        ${CMAKE_CURRENT_LIST_DIR}/Steering.cpp
        ${CMAKE_CURRENT_LIST_DIR}/balance_bot.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_steering.cpp
        ${CMAKE_CURRENT_LIST_DIR}/failsafe.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_rtl.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_hold.cpp
        ${CMAKE_CURRENT_LIST_DIR}/system.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Rover.h
        ${CMAKE_CURRENT_LIST_DIR}/cruise_learn.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode.h
        ${CMAKE_CURRENT_LIST_DIR}/AP_Rally.h
)

# add current directory to the compiler included directories when compiling the given target.
target_include_directories(${CURRENT_EXE_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

