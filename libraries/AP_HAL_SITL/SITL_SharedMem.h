#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>

/*
  shared memory segment allowing multiple SITL instances to share
  simulated clock state. Each instance writes its own sim_time_us into
  a slot indexed by instance number. Readers can check peer times to
  determine relative clock positions.

  The segment is named /ardupilot_sitl_shmem and persists until the
  last instance unmaps it and calls shm_unlink().

  No locking is required: each slot has exactly one writer and the
  64-bit time writes are naturally atomic on x86/ARM64. A pid field
  per slot lets readers detect stale entries from crashed instances.
*/

#define AP_SITL_SHMEM_MAX_INSTANCES 16
#define AP_SITL_SHMEM_MAGIC         0x4150534D  // "APSM"
#define AP_SITL_SHMEM_NAME          "/ardupilot_sitl_shmem"

/*
  layout of the shared memory segment
*/
struct AP_SITL_ShmData {
    uint32_t magic;
    uint32_t version;
    struct {
        uint64_t sim_time_us;   // simulated time in microseconds
        pid_t    pid;           // pid of owning process (0 = slot unused)
    } instance[AP_SITL_SHMEM_MAX_INSTANCES];
};

class AP_SITL_SharedMem {
public:
    AP_SITL_SharedMem();
    ~AP_SITL_SharedMem();

    /*
      initialise shared memory for the given instance number.
      Must be called once on startup before update() or get_time_us().
    */
    bool init(uint8_t instance_id);

    /*
      update this instance's clock in the shared segment.
      Call this periodically (e.g. every scheduler tick).
    */
    void update(uint64_t time_us);

    /*
      return the last published sim_time_us for the given instance.
      Returns 0 if the instance has no active entry or shm is not initialised.
    */
    uint64_t get_time_us(uint8_t instance_id) const;

    /*
      return true if the given instance slot appears active
      (pid is alive and time is non-zero)
    */
    bool instance_active(uint8_t instance_id) const;

    /*
      return the number of currently active instances
    */
    uint8_t get_instance_count() const;

    bool is_initialised() const { return _data != nullptr; }

private:
    AP_SITL_ShmData *_data;
    int              _fd;
    uint8_t          _instance_id;
    bool             _created;  // true if we created the shm segment

    void _clear_slot();
    void _cleanup();
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
