# After the NEW firmware is flashed: reset all params to defaults (so defaults.parm
# is what the board comes up with), then reboot. Both are COMMAND_LONGs, which the
# vehicle acks even while the param IO thread is starved. WIPES calibrations.
import sys, time
from pymavlink import mavutil
mav=mavutil.mavlink
link=sys.argv[1] if len(sys.argv)>1 else 'udpin:127.0.0.1:14550'
m=mavutil.mavlink_connection(link); hb=m.wait_heartbeat(timeout=10)
if not hb: print("NO HEARTBEAT"); sys.exit(2)
ts,tc=hb.get_srcSystem(),1
def cmd(name, cid, p1):
    m.mav.command_long_send(ts,tc,cid,0,p1,0,0,0,0,0,0); t0=time.time()
    while time.time()-t0<8:
        r=m.recv_match(type='COMMAND_ACK',blocking=True,timeout=1)
        if r and r.command==cid: print("%s -> ack result %d"%(name,r.result)); return r.result
    print("%s -> NO ACK"%name); return None
if cmd("PREFLIGHT_STORAGE(reset to defaults)", mav.MAV_CMD_PREFLIGHT_STORAGE, 2) == 0:
    time.sleep(2); cmd("PREFLIGHT_REBOOT", mav.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
