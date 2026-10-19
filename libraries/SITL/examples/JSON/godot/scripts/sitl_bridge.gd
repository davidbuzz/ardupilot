class_name ArduPilotSITLBridge
extends RefCounted
## ArduPilot SITL JSON interface, see libraries/SITL/examples/JSON/readme.md
##
## SITL sends a binary servo packet for every frame and waits for the
## physics backend to reply with the resulting vehicle state as JSON.

const MAGIC_16 := 18458
const MAGIC_32 := 29569

var udp := PacketPeerUDP.new()
var frame_rate := 0
var frame_count := -1
var pwm := PackedInt32Array()
var sitl_ip := ""
var sitl_port := 0
var last_reply := PackedByteArray()


func listen(port: int) -> Error:
    return udp.bind(port, "*")


## wait up to timeout_us for a servo packet. Returns true if a new frame
## arrived, false on timeout. A repeated frame (SITL resends if it gets no
## reply) is answered with the previous reply and not reported.
func wait_for_servos(timeout_us: int) -> bool:
    var deadline := Time.get_ticks_usec() + timeout_us
    while true:
        while udp.get_available_packet_count() > 0:
            var pkt := udp.get_packet()
            var ip := udp.get_packet_ip()
            var port := udp.get_packet_port()
            if not _decode(pkt):
                continue
            sitl_ip = ip
            sitl_port = port
            udp.set_dest_address(sitl_ip, sitl_port)
            return true
        if Time.get_ticks_usec() >= deadline:
            return false
        OS.delay_usec(20)
    return false


func _decode(pkt: PackedByteArray) -> bool:
    if pkt.size() < 8:
        return false
    var magic := pkt.decode_u16(0)
    var channels := 16 if magic == MAGIC_16 else (32 if magic == MAGIC_32 else 0)
    if channels == 0 or pkt.size() != 8 + 2 * channels:
        push_warning("ignoring bad SITL packet: magic %d, %d bytes" % [magic, pkt.size()])
        return false
    var count := pkt.decode_u32(4)
    if count == frame_count and not last_reply.is_empty():
        udp.put_packet(last_reply)
        return false
    frame_rate = pkt.decode_u16(2)
    frame_count = count
    pwm.resize(channels)
    for i in channels:
        pwm[i] = pkt.decode_u16(8 + 2 * i)
    return true


func send_state(timestamp: float, s: Dictionary) -> void:
    var g: Vector3 = s["gyro"]
    var a: Vector3 = s["accel_body"]
    var p: Vector3 = s["position"]
    var v: Vector3 = s["velocity"]
    var q: Quaternion = s["quaternion"]
    # hand-formatted to keep the key order fixed, which SITL's simple
    # parser relies on, and to bracket the line with newlines
    var line := ("\n{\"timestamp\":%.6f,\"imu\":{\"gyro\":[%.6f,%.6f,%.6f],\"accel_body\":[%.6f,%.6f,%.6f]},"
            + "\"position\":[%.4f,%.4f,%.4f],\"quaternion\":[%.8f,%.8f,%.8f,%.8f],"
            + "\"velocity\":[%.5f,%.5f,%.5f]}\n") % [
            timestamp, g.x, g.y, g.z, a.x, a.y, a.z, p.x, p.y, p.z,
            q.w, q.x, q.y, q.z, v.x, v.y, v.z]
    last_reply = line.to_ascii_buffer()
    udp.put_packet(last_reply)
