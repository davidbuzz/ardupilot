#!/usr/bin/env bash
# Tools/scripts/test_sitl_multi_instance.sh
#
# Launch three ArduCopter SITL instances in parallel at speedup=100 and
# verify that they remain clock-synchronised via shared memory.
#
# Usage:
#   Tools/scripts/test_sitl_multi_instance.sh [binary]
#
# The optional argument overrides the default binary path.
#
# Exit codes:
#   0  all instances completed within expected sim-time bounds
#   1  one or more instances failed or timed out
#
# AP_FLAKE8_CLEAN (N/A - this is a shell script)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BINARY="${1:-${REPO_ROOT}/build/sitl/bin/arducopter}"
SPEEDUP=100
NUM_INSTANCES=3
# How many simulated seconds to run before checking sync
SIM_DURATION_S=30
# Allow up to this many seconds of wall-clock time for the test
WALL_TIMEOUT_S=120

if [ ! -x "${BINARY}" ]; then
    echo "ERROR: binary not found or not executable: ${BINARY}"
    exit 1
fi

TMPDIR="$(mktemp -d /tmp/sitl_multi_XXXXXX)"
trap 'cleanup' EXIT INT TERM

cleanup() {
    echo "Cleaning up..."
    # Kill any remaining SITL processes
    for pid_file in "${TMPDIR}"/instance_*.pid; do
        [ -f "${pid_file}" ] || continue
        pid=$(cat "${pid_file}")
        kill "${pid}" 2>/dev/null || true
    done
    # Remove shared memory segment if it exists
    rm -f /dev/shm/ardupilot_sitl_shmem 2>/dev/null || true
    python3 -c "
import ctypes, ctypes.util
try:
    rt = ctypes.CDLL(ctypes.util.find_library('rt') or 'librt.so.1')
    rt.shm_unlink(b'/ardupilot_sitl_shmem')
except Exception:
    pass
" 2>/dev/null || true
    rm -rf "${TMPDIR}"
}

echo "=== SITL multi-instance clock-sync test ==="
echo "Binary:       ${BINARY}"
echo "Speedup:      ${SPEEDUP}"
echo "Instances:    ${NUM_INSTANCES}"
echo "Sim duration: ${SIM_DURATION_S} s (simulated)"
echo "Tmp dir:      ${TMPDIR}"
echo ""

# Export fleet size so each instance knows how many peers to wait for
export SITL_INSTANCE_COUNT=${NUM_INSTANCES}

# Launch all instances in the background
for i in $(seq 0 $((NUM_INSTANCES - 1))); do
    inst_dir="${TMPDIR}/instance_${i}"
    mkdir -p "${inst_dir}"

    "${BINARY}" \
        --instance "${i}" \
        --speedup "${SPEEDUP}" \
        --wipe-eeprom \
        --home "-35.363261,149.165230,584,353" \
        --model "+" \
        --uartA "tcp:0" \
        >"${inst_dir}/stdout.log" 2>"${inst_dir}/stderr.log" &

    echo "${!}" > "${TMPDIR}/instance_${i}.pid"
    echo "Launched instance ${i} (pid ${!})"
done

echo ""
echo "Waiting for instances to settle (5 s wall time)..."
sleep 5

# Poll for sync: each instance should have a non-zero sim_time_us
# We use a Python helper to read the shared memory segment directly.
POLL_SCRIPT="${TMPDIR}/check_sync.py"
cat > "${POLL_SCRIPT}" << 'PYEOF'
#!/usr/bin/env python3
"""
Read the /ardupilot_sitl_shmem POSIX shared memory segment and report
the sim_time_us for each slot.  Exits with status 0 if all expected
instances are within 1 simulated second of each other, 1 otherwise.

Usage: check_sync.py <num_instances> <min_sim_time_us>
"""
import sys
import ctypes
import ctypes.util
import struct
import mmap
import os

MAX_INSTANCES = 16
MAGIC = 0x4150534D
SHM_NAME = "/ardupilot_sitl_shmem"

num_instances = int(sys.argv[1])
min_sim_time_us = int(sys.argv[2])

# Open the shared memory segment via /dev/shm (Linux)
shm_path = "/dev/shm" + SHM_NAME
if not os.path.exists(shm_path):
    print(f"ERROR: {shm_path} not found")
    sys.exit(1)

with open(shm_path, "rb") as f:
    # Layout: magic(4) version(4) total_instances(4) _pad(4)
    #   then MAX_INSTANCES * (sim_time_us(8) pid(4) _pad(4))
    header = struct.unpack("<IIII", f.read(16))
    magic, version, total_instances, _ = header
    if magic != MAGIC:
        print(f"ERROR: bad magic 0x{magic:08X}")
        sys.exit(1)
    slots = []
    for _ in range(MAX_INSTANCES):
        data = f.read(16)
        sim_time_us, pid, pad = struct.unpack("<QII", data)
        slots.append((sim_time_us, pid))

print(f"total_instances={total_instances}")
for i, (t, pid) in enumerate(slots[:num_instances]):
    print(f"  instance[{i}]: sim_time_us={t} pid={pid}")

times = [slots[i][0] for i in range(num_instances)]
if any(t < min_sim_time_us for t in times):
    print(f"FAIL: not all instances reached min_sim_time_us={min_sim_time_us}")
    sys.exit(1)

skew = max(times) - min(times)
print(f"Skew: {skew} us ({skew/1e6:.3f} s simulated)")
# Allow up to 1 simulated second of skew
if skew > 1_000_000:
    print(f"FAIL: skew {skew} us exceeds 1 s limit")
    sys.exit(1)

print("PASS: all instances in sync")
sys.exit(0)
PYEOF

# Wait for instances to accumulate SIM_DURATION_S of simulated time
MIN_SIM_TIME_US=$((SIM_DURATION_S * 1000000))

echo "Polling for ${SIM_DURATION_S} simulated seconds (wall timeout ${WALL_TIMEOUT_S} s)..."

START_WALL=$(date +%s)
while true; do
    NOW_WALL=$(date +%s)
    ELAPSED=$((NOW_WALL - START_WALL))
    if [ "${ELAPSED}" -ge "${WALL_TIMEOUT_S}" ]; then
        echo "ERROR: wall-clock timeout after ${ELAPSED} s"
        exit 1
    fi

    # Check that all instances are still alive
    all_alive=1
    for i in $(seq 0 $((NUM_INSTANCES - 1))); do
        pid=$(cat "${TMPDIR}/instance_${i}.pid")
        if ! kill -0 "${pid}" 2>/dev/null; then
            echo "ERROR: instance ${i} (pid ${pid}) died"
            cat "${TMPDIR}/instance_${i}/stderr.log" | tail -20
            exit 1
        fi
    done

    # Check sync state
    if python3 "${POLL_SCRIPT}" "${NUM_INSTANCES}" "${MIN_SIM_TIME_US}" 2>&1; then
        echo ""
        echo "=== PASS: all ${NUM_INSTANCES} instances clock-synchronised at speedup=${SPEEDUP} ==="
        exit 0
    fi

    sleep 2
done
