#!/usr/bin/env python3
"""
SERIAL5 (PIOUART2) loopback validation for Pico2.
GPIO21 (TX) → GPIO27 (RX) loopback wiring required.
Device ID: 105 (SERIAL5 in SERIAL_CONTROL)
"""

import argparse
import time
import sys
from pymavlink import mavutil

def test_serial5_loopback(port, duration=60, verbose=False):
    """
    Comprehensive SERIAL5 loopback test via MAVLink SERIAL_CONTROL.
    
    Args:
        port: MAVLink device port (e.g., '/dev/ttyACM1')
        duration: Test duration in seconds
        verbose: Print detailed per-packet results
    
    Returns:
        dict with test results: ok, bad, rx_zero, total_tests
    """
    
    print(f"[SERIAL5] Connecting to MAVLink on {port}...")
    try:
        m = mavutil.mavlink_connection(port, baud=115200, source_system=255, source_component=190)
        m.wait_heartbeat(timeout=10)
        print(f"[SERIAL5] ✓ Heartbeat received")
    except Exception as e:
        print(f"[SERIAL5] ✗ Failed to connect: {e}")
        return None
    
    # Test parameters
    device_id = 105  # SERIAL5
    flags = (1 << 1) | (1 << 2)  # RESPOND | EXCLUSIVE
    test_payloads = [
        ([0x55] * 70, "0x55 (alternating bits)"),
        ([0xAA] * 70, "0xAA (alternating bits, inverted)"),
        ([ord('A')] * 70, "0x41 ('A' character)"),
        ([0xFF] * 70, "0xFF (all ones)"),
        ([0x00] * 70, "0x00 (all zeros)"),
        (list(range(70)), "Incrementing 0-69 mod 256"),
    ]
    
    results = {
        'ok': 0,
        'bad': 0,
        'rx_zero': 0,
        'rx_corrupted': 0,
        'total_tests': 0,
        'by_payload': {}
    }
    
    start_time = time.time()
    test_idx = 0
    
    print(f"\n[SERIAL5] Starting {duration}s loopback test...")
    print(f"[SERIAL5] Device ID: {device_id} (SERIAL5/PIOUART2)")
    print(f"[SERIAL5] GPIO21 (TX) → GPIO27 (RX) via jumper\n")
    
    while time.time() - start_time < duration:
        payload, desc = test_payloads[test_idx % len(test_payloads)]
        test_idx += 1
        
        # Send SERIAL_CONTROL packet
        m.mav.serial_control_send(device_id, flags, 0, 115200, len(payload), payload)
        
        # Wait briefly for reply
        reply_timeout = time.time() + 0.3
        reply_received = False
        reply_len = 0
        reply_data = None
        
        while time.time() < reply_timeout:
            msg = m.recv_match(type='SERIAL_CONTROL', blocking=False)
            if msg and msg.device == device_id:
                reply_received = True
                reply_len = msg.count
                reply_data = bytes(msg.data[:reply_len]) if reply_len > 0 else None
                break
            time.sleep(0.002)
        
        results['total_tests'] += 1
        payload_bytes = bytes(payload)
        
        # Categorize result
        if not reply_received or reply_len == 0:
            results['rx_zero'] += 1
            status = "✗ NO_RX"
        elif reply_data == payload_bytes:
            results['ok'] += 1
            status = "✓ MATCH"
        elif payload_bytes[0:1] in reply_data or any(b in reply_data for b in payload_bytes[:10]):
            results['bad'] += 1
            status = "⚠ PARTIAL"
        else:
            results['rx_corrupted'] += 1
            status = "✗ CORRUPT"
        
        # Track per-payload stats
        if desc not in results['by_payload']:
            results['by_payload'][desc] = {'ok': 0, 'bad': 0, 'rx_zero': 0, 'corrupted': 0}
        
        if status == "✓ MATCH":
            results['by_payload'][desc]['ok'] += 1
        elif status == "⚠ PARTIAL":
            results['by_payload'][desc]['bad'] += 1
        elif status == "✗ NO_RX":
            results['by_payload'][desc]['rx_zero'] += 1
        else:
            results['by_payload'][desc]['corrupted'] += 1
        
        if verbose or test_idx % 20 == 0:
            elapsed = time.time() - start_time
            print(f"[{elapsed:6.2f}s] Test {test_idx:3d}: {status:12s} len={reply_len:2d}  {desc}")
        
        time.sleep(0.01)
    
    # Print summary
    elapsed = time.time() - start_time
    ok_pct = 100 * results['ok'] / results['total_tests'] if results['total_tests'] > 0 else 0
    
    print(f"\n{'='*70}")
    print(f"[SERIAL5] Test Summary ({elapsed:.1f}s, {results['total_tests']} packets)")
    print(f"{'='*70}")
    print(f"  ✓ MATCH         : {results['ok']:3d}/{results['total_tests']} ({ok_pct:5.1f}%)")
    print(f"  ⚠ PARTIAL       : {results['bad']:3d}/{results['total_tests']} ({100*results['bad']/results['total_tests']:5.1f}%)")
    print(f"  ✗ NO_RX         : {results['rx_zero']:3d}/{results['total_tests']} ({100*results['rx_zero']/results['total_tests']:5.1f}%)")
    print(f"  ✗ CORRUPTED     : {results['rx_corrupted']:3d}/{results['total_tests']} ({100*results['rx_corrupted']/results['total_tests']:5.1f}%)")
    print(f"{'='*70}")
    
    print(f"\n[SERIAL5] Per-payload breakdown:")
    for desc, stats in results['by_payload'].items():
        total = stats['ok'] + stats['bad'] + stats['rx_zero'] + stats['corrupted']
        if total > 0:
            pct = 100 * stats['ok'] / total
            print(f"  {desc:35s}: {stats['ok']:2d}/{total:2d} ok ({pct:5.1f}%)")
    
    # Success criteria
    print(f"\n[SERIAL5] Result:")
    if results['ok'] >= results['total_tests'] * 0.95:
        print(f"  ✓ PASS: {results['ok']}/{results['total_tests']} exact matches (≥95%)")
        return results
    elif results['ok'] >= results['total_tests'] * 0.80:
        print(f"  ⚠ PARTIAL: {results['ok']}/{results['total_tests']} exact matches (80-95%)")
        print(f"    → Same pattern as SERIAL3/4 before final fixes; may be GCS aggregation timing")
        return results
    else:
        print(f"  ✗ FAIL: Only {results['ok']}/{results['total_tests']} matches (<80%)")
        if results['rx_zero'] > results['total_tests'] * 0.5:
            print(f"    → High NO_RX rate: Check GPIO21→GPIO27 jumper wiring!")
        return results

def main():
    parser = argparse.ArgumentParser(description="SERIAL5 loopback test")
    parser.add_argument('--port', default='/dev/ttyACM1', help='MAVLink device port')
    parser.add_argument('--duration', type=int, default=60, help='Test duration (seconds)')
    parser.add_argument('--verbose', action='store_true', help='Verbose per-packet output')
    args = parser.parse_args()
    
    results = test_serial5_loopback(args.port, args.duration, args.verbose)
    
    if results is None:
        sys.exit(1)
    elif results['ok'] < results['total_tests'] * 0.80:
        sys.exit(2)
    else:
        sys.exit(0)

if __name__ == '__main__':
    main()
