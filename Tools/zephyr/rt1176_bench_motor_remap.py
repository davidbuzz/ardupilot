# Align software motor numbering to the physical layout without re-plugging.
#   python3 rt1176_bench_motor_remap.py "1:FR,4:FL,2:RR,3:RL"      (plug:position)
#   python3 rt1176_bench_motor_remap.py --verify                     (just the sweep)
# QUAD X: Motor1 (function 33) front-right CCW, Motor2 (34) rear-left CCW,
#         Motor3 (35) front-left CW,   Motor4 (36) rear-right CW.
# The sweep afterwards must go FR -> RR -> RL -> FL (sequence 1..4).
import sys, time, glob
from pymavlink import mavutil
mav=mavutil.mavlink
FUNC={'FR':33,'RL':34,'FL':35,'RR':36}; NAME={33:'Motor1 front-right CCW',34:'Motor2 rear-left CCW',35:'Motor3 front-left CW',36:'Motor4 rear-right CW'}
p=glob.glob('/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*-if-mavlink')[0]
m=mavutil.mavlink_connection(p,baud=115200); hb=m.wait_heartbeat(timeout=20); ts=hb.get_srcSystem()
def read(n):
    for i in range(3):
        m.mav.param_request_read_send(ts,1,n.encode(),-1); t1=time.time()
        while time.time()-t1<2:
            x=m.recv_match(type='PARAM_VALUE',blocking=True,timeout=1)
            if x and x.param_id.rstrip('\x00')==n: return x.param_value
def setp(n,v):
    for i in range(3):
        m.mav.param_set_send(ts,1,n.encode(),float(v),mav.MAV_PARAM_TYPE_INT16); t1=time.time()
        while time.time()-t1<2:
            y=m.recv_match(type='PARAM_VALUE',blocking=True,timeout=1)
            if y and y.param_id.rstrip('\x00')==n: return y.param_value
print("current:", {n:int(read('SERVO%d_FUNCTION'%n) or 0) for n in (1,2,3,4)})
if len(sys.argv)>1 and sys.argv[1]!='--verify':
    layout={int(a.split(':')[0]):a.split(':')[1].strip().upper() for a in sys.argv[1].split(',')}
    assert sorted(layout)==[1,2,3,4] and sorted(layout.values())==['FL','FR','RL','RR'], layout
    for plug,pos in layout.items():
        f=FUNC[pos]; got=setp('SERVO%d_FUNCTION'%plug,f); print("SERVO%d_FUNCTION -> %d (%s): %s"%(plug,f,NAME[f],got))
    time.sleep(1)
print("verify sweep: sequence 1..4 must spin FR, RR, RL, FL")
m.mav.request_data_stream_send(ts,1,mav.MAV_DATA_STREAM_RC_CHANNELS,10,1)
for seq,exp in ((1,'FRONT-RIGHT'),(2,'REAR-RIGHT'),(3,'REAR-LEFT'),(4,'FRONT-LEFT')):
    print("%s  >>> sequence %d, expect %s, in 2 s"%(time.strftime('%H:%M:%S'),seq,exp), flush=True); time.sleep(2)
    m.mav.command_long_send(ts,1,mav.MAV_CMD_DO_MOTOR_TEST,0,seq,1,1200,4,1,0,0); t0=time.time(); driven=set()
    while time.time()-t0<4.5:
        r=m.recv_match(type='SERVO_OUTPUT_RAW',blocking=True,timeout=0.5)
        if r:
            for ch in range(1,5):
                if getattr(r,'servo%d_raw'%ch)>1050: driven.add(ch)
    print("      drove plug %s"%sorted(driven), flush=True); time.sleep(5)
m.mav.request_data_stream_send(ts,1,mav.MAV_DATA_STREAM_RC_CHANNELS,0,0); print("done")
