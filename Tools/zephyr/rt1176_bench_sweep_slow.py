# Announced motor sweep for the lead-order check: sequence 1..4 at 1200 us, 4 s
# each, 5 s gaps. The human calls out physical position and direction per step.
import time, glob, sys
from pymavlink import mavutil
mav=mavutil.mavlink
pwm=int(sys.argv[1]) if len(sys.argv)>1 else 1200
p=glob.glob('/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*-if-mavlink')[0]
m=mavutil.mavlink_connection(p,baud=115200); hb=m.wait_heartbeat(timeout=20); ts=hb.get_srcSystem()
m.mav.request_data_stream_send(ts,1,mav.MAV_DATA_STREAM_RC_CHANNELS,10,1)
pos={1:'FRONT-RIGHT (out1, CCW)',2:'REAR-RIGHT (out4, CW)',3:'REAR-LEFT (out2, CCW)',4:'FRONT-LEFT (out3, CW)'}
for seq in (1,2,3,4):
    print("%s  >>> SEQUENCE %d  (ArduPilot expects %s)  starting in 2 s"%(time.strftime('%H:%M:%S'),seq,pos[seq]), flush=True); time.sleep(2)
    m.mav.command_long_send(ts,1,mav.MAV_CMD_DO_MOTOR_TEST,0,seq,1,pwm,4,1,0,0); t0=time.time(); driven=set()
    while time.time()-t0<4.5:
        r=m.recv_match(type='SERVO_OUTPUT_RAW',blocking=True,timeout=0.5)
        if r:
            for ch in range(1,5):
                if getattr(r,'servo%d_raw'%ch)>1050: driven.add(ch)
    print("%s      sequence %d drove output %s"%(time.strftime('%H:%M:%S'),seq,sorted(driven)), flush=True); time.sleep(5)
m.mav.request_data_stream_send(ts,1,mav.MAV_DATA_STREAM_RC_CHANNELS,0,0)
print("sweep done")
