# add source and header files when compiling the given target.
# although it is not necessary to add header files, we also do that for easing the management of IDE.
target_sources(${CURRENT_EXE_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/radio.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RC_Channel.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Parameters.h
        ${CMAKE_CURRENT_LIST_DIR}/RC_Channel.h
        ${CMAKE_CURRENT_LIST_DIR}/config.h
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Mavlink.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_scan.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_guided.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Log.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Tracker.h
        ${CMAKE_CURRENT_LIST_DIR}/Parameters.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Tracker.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_servotest.cpp
        ${CMAKE_CURRENT_LIST_DIR}/version.h
        ${CMAKE_CURRENT_LIST_DIR}/tracking.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AP_Arming.h
        ${CMAKE_CURRENT_LIST_DIR}/mode_manual.cpp
        ${CMAKE_CURRENT_LIST_DIR}/servos.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode.cpp
        ${CMAKE_CURRENT_LIST_DIR}/defines.h
        ${CMAKE_CURRENT_LIST_DIR}/sensors.cpp
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Tracker.h
        ${CMAKE_CURRENT_LIST_DIR}/GCS_Mavlink.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode_auto.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Tracker.cpp
        ${CMAKE_CURRENT_LIST_DIR}/system.cpp
        ${CMAKE_CURRENT_LIST_DIR}/mode.h
)

# add current directory to the compiler included directories when compiling the given target.
target_include_directories(${CURRENT_EXE_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

