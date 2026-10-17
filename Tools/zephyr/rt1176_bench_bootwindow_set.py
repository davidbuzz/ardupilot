# Land SCHED_LOOP_RATE 300 in the boot window, before the main loop saturates and
# starves the IO thread that answers PARAM_SET. Run right after the board reboots.
import time, sys
from pymavlink import mavutil
link=sys.argv[1] if len(sys.argv)>1 else 'udpin:127.0.0.1:14550'
m=mavutil.mavlink_connection(link)
hb=m.wait_heartbeat(timeout=90)
if not hb: print("NO HEARTBEAT in 90s"); sys.exit(2)
ts,tc=hb.get_srcSystem(),1; t0=time.time(); ack=None; n=0
print("heartbeat at +0.0s, hammering PARAM_SET", flush=True)
while time.time()-t0<45 and ack is None:
    m.mav.param_set_send(ts,tc,b'SCHED_LOOP_RATE',300.0,mavutil.mavlink.MAV_PARAM_TYPE_INT16); n+=1
    r=m.recv_match(type='PARAM_VALUE',blocking=True,timeout=0.5)
    if r and r.param_id.rstrip('\x00')=='SCHED_LOOP_RATE': ack=r.param_value
print("ack=%s after %d sends, %.1fs" % (ack,n,time.time()-t0))
