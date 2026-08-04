#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SITL_SharedMem.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>

AP_SITL_SharedMem::AP_SITL_SharedMem() :
    _data(nullptr),
    _fd(-1),
    _instance_id(0),
    _created(false)
{
}

AP_SITL_SharedMem::~AP_SITL_SharedMem()
{
    _cleanup();
}

bool AP_SITL_SharedMem::init(uint8_t instance_id, uint8_t total_instances)
{
    if (instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        fprintf(stderr, "SITL_SharedMem: instance_id %u >= max %u\n",
                instance_id, AP_SITL_SHMEM_MAX_INSTANCES);
        return false;
    }

    _instance_id = instance_id;

    // try to open existing segment first
    _fd = shm_open(AP_SITL_SHMEM_NAME, O_RDWR | O_CREAT | O_EXCL,
                   S_IRUSR | S_IWUSR);
    if (_fd >= 0) {
        // we created the segment - initialise it
        _created = true;
        if (ftruncate(_fd, sizeof(AP_SITL_ShmData)) < 0) {
            perror("SITL_SharedMem: ftruncate");
            _cleanup();
            return false;
        }
    } else {
        // segment exists - open it
        _fd = shm_open(AP_SITL_SHMEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
        if (_fd < 0) {
            perror("SITL_SharedMem: shm_open");
            return false;
        }
        _created = false;
    }

    _data = static_cast<AP_SITL_ShmData *>(
        mmap(nullptr, sizeof(AP_SITL_ShmData),
             PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0));

    if (_data == MAP_FAILED) {
        perror("SITL_SharedMem: mmap");
        _data = nullptr;
        _cleanup();
        return false;
    }

    if (_created) {
        memset(_data, 0, sizeof(AP_SITL_ShmData));
        _data->magic   = AP_SITL_SHMEM_MAGIC;
        _data->version = 1;
    } else if (_data->magic != AP_SITL_SHMEM_MAGIC) {
        fprintf(stderr, "SITL_SharedMem: bad magic 0x%08X in existing segment\n",
                _data->magic);
        _cleanup();
        return false;
    }

    // publish total_instances: use max of current value and what we know
    if (total_instances > _data->total_instances) {
        _data->total_instances = total_instances;
    }

    // claim our slot
    _data->instance[_instance_id].pid         = getpid();
    _data->instance[_instance_id].sim_time_us = 0;

    return true;
}

void AP_SITL_SharedMem::update(uint64_t time_us)
{
    if (_data == nullptr) {
        return;
    }
    _data->instance[_instance_id].sim_time_us = time_us;
}

uint64_t AP_SITL_SharedMem::get_time_us(uint8_t instance_id) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return 0;
    }
    return _data->instance[instance_id].sim_time_us;
}

bool AP_SITL_SharedMem::instance_active(uint8_t instance_id) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return false;
    }
    const pid_t pid = _data->instance[instance_id].pid;
    if (pid <= 0) {
        return false;
    }
    // check pid is alive by sending signal 0
    return kill(pid, 0) == 0;
}

uint8_t AP_SITL_SharedMem::get_instance_count() const
{
    if (_data == nullptr) {
        return 0;
    }
    uint8_t count = 0;
    for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
        if (instance_active(i)) {
            count++;
        }
    }
    return count;
}

/*
  spin-wait barrier: pause until all expected peer instances have
  published a sim_time_us >= (sim_time_us - max_skew_us).

  At high speedup each instance advances sim time much faster than wall
  time. Without coordination the instances can diverge by arbitrarily
  many simulated seconds. This function is called from
  Aircraft::sync_frame_time() after every simulation step; it holds the
  fastest instance back until the slowest catches up within max_skew_us.

  A wall-clock timeout (default 5 s) prevents an indefinite stall when a
  peer has crashed.
*/
bool AP_SITL_SharedMem::sync_with_peers(uint64_t sim_time_us,
                                         uint64_t max_skew_us,
                                         uint64_t timeout_us)
{
    if (_data == nullptr || _data->total_instances <= 1) {
        return true;
    }

    const uint32_t n = _data->total_instances;
    if (n > AP_SITL_SHMEM_MAX_INSTANCES) {
        return true;
    }

    // minimum sim_time we are willing to be ahead of peers
    const uint64_t min_peer_time = (sim_time_us > max_skew_us)
                                    ? sim_time_us - max_skew_us
                                    : 0;

    // get wall-clock start for timeout
    struct timeval tv_start;
    gettimeofday(&tv_start, nullptr);

    while (true) {
        bool all_ok = true;
        for (uint32_t i = 0; i < n; i++) {
            if (i == _instance_id) {
                continue;
            }
            const pid_t pid = _data->instance[i].pid;
            if (pid <= 0) {
                // slot not yet registered; wait for it
                all_ok = false;
                break;
            }
            // if the peer has crashed, stop waiting for it
            if (kill(pid, 0) != 0) {
                // dead peer - clear its slot so future calls skip it
                _data->instance[i].pid = 0;
                continue;
            }
            if (_data->instance[i].sim_time_us < min_peer_time) {
                all_ok = false;
                break;
            }
        }
        if (all_ok) {
            return true;
        }

        // check wall-clock timeout
        struct timeval tv_now;
        gettimeofday(&tv_now, nullptr);
        const uint64_t elapsed_us =
            (uint64_t)(tv_now.tv_sec  - tv_start.tv_sec)  * 1000000ULL +
            (uint64_t)(tv_now.tv_usec - tv_start.tv_usec);
        if (elapsed_us > timeout_us) {
            fprintf(stderr,
                    "SITL_SharedMem: sync timeout after %.1f s waiting for "
                    "peers (our sim_time_us=%llu)\n",
                    elapsed_us * 1e-6,
                    (unsigned long long)sim_time_us);
            return false;
        }

        // short sleep to avoid hammering the cache line
        usleep(100);
    }
}

void AP_SITL_SharedMem::_clear_slot()
{
    if (_data == nullptr) {
        return;
    }
    _data->instance[_instance_id].sim_time_us = 0;
    _data->instance[_instance_id].pid         = 0;
}

void AP_SITL_SharedMem::_cleanup()
{
    _clear_slot();

    if (_data != nullptr && _data != MAP_FAILED) {
        munmap(_data, sizeof(AP_SITL_ShmData));
        _data = nullptr;
    }

    if (_fd >= 0) {
        close(_fd);
        _fd = -1;
    }

    // only the creator tries to unlink; if other instances are still
    // alive it is harmless because they hold their own mappings
    if (_created) {
        shm_unlink(AP_SITL_SHMEM_NAME);
        _created = false;
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
