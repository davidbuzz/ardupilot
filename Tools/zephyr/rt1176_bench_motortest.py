# Per-motor spin-up test, PROPS OFF. For each motor 1..4 and each PWM in the list:
# DO_MOTOR_TEST (throttle type 1 = PWM), hold 2 s, record SERVO_OUTPUT_RAW for that
# channel. Prints a table; the human reports which motor turned at which PWM.
import sys, time, glob
from pymavlink import mavutil
mav=mavutil.mavlink
link=sys.argv[1] if len(sys.argv)>1 else glob.glob('/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*-if-mavlink')[0]
pwms=[int(x) for x in (sys.argv[2].split(',') if len(sys.argv)>2 else ['1100','1150','1200'])]
m=mavutil.mavlink_connection(link,baud=115200); hb=m.wait_heartbeat(timeout=10); ts=hb.get_srcSystem()
m.mav.request_data_stream_send(ts,1,mav.MAV_DATA_STREAM_RC_CHANNELS,10,1)   # SERVO_OUTPUT_RAW rides on this stream
for motor in (1,2,3,4):
    for pwm in pwms:
        m.mav.command_long_send(ts,1,mav.MAV_CMD_DO_MOTOR_TEST,0,motor,1,pwm,2,1,0,0)
        ack=None; t0=time.time(); outs=[]
        while time.time()-t0<2.6:
            r=m.recv_match(type=['COMMAND_ACK','SERVO_OUTPUT_RAW'],blocking=True,timeout=0.5)
            if r is None: continue
            if r.get_type()=='COMMAND_ACK' and r.command==mav.MAV_CMD_DO_MOTOR_TEST: ack=r.result
            elif r.get_type()=='SERVO_OUTPUT_RAW': outs.append(getattr(r,'servo%d_raw'%motor))
        print("motor %d  pwm %d  ack=%s  servo%d_raw seen: %s" % (motor,pwm,ack,motor,sorted(set(outs))), flush=True)
        time.sleep(1.0)
m.mav.request_data_stream_send(ts,1,mav.MAV_DATA_STREAM_RC_CHANNELS,0,0)
print("done - which motors turned, and at which PWM?")
