# Compass calibration over the UDP link: start, report progress and fitness,
# accept when complete. The human rotates the vehicle through all axes.
#   python3 rt1176_bench_compass_cal.py [udpin:127.0.0.1:14550]
import sys, time
from pymavlink import mavutil
mav=mavutil.mavlink
link=sys.argv[1] if len(sys.argv)>1 else 'udpin:127.0.0.1:14550'
m=mavutil.mavlink_connection(link); hb=m.wait_heartbeat(timeout=10); ts=hb.get_srcSystem()
# DO_START_MAG_CAL: mask 0 = all compasses, retry 0, autosave 1, delay 0, autoreboot 0
m.mav.command_long_send(ts,1,mav.MAV_CMD_DO_START_MAG_CAL,0,0,0,1,0,0,0,0)
t0=time.time(); last=-1; done=False
while time.time()-t0<240 and not done:
    r=m.recv_match(type=['COMMAND_ACK','MAG_CAL_PROGRESS','MAG_CAL_REPORT','STATUSTEXT'],blocking=True,timeout=1)
    if r is None: continue
    t=r.get_type()
    if t=='COMMAND_ACK' and r.command==mav.MAV_CMD_DO_START_MAG_CAL: print("start ack", r.result, flush=True)
    elif t=='MAG_CAL_PROGRESS' and r.completion_pct!=last:
        last=r.completion_pct; print("  compass %d: %d%% (status %d)"%(r.compass_id,r.completion_pct,r.cal_status), flush=True)
    elif t=='MAG_CAL_REPORT':
        print("REPORT compass %d: status %d fitness %.1f ofs %.0f/%.0f/%.0f"%(r.compass_id,r.cal_status,r.fitness,r.ofs_x,r.ofs_y,r.ofs_z), flush=True)
        if r.cal_status==4:   # MAG_CAL_SUCCESS
            m.mav.command_long_send(ts,1,mav.MAV_CMD_DO_ACCEPT_MAG_CAL,0,0,0,0,0,0,0,0); print("accepted; reboot the vehicle before arming", flush=True); done=True
        elif r.cal_status>4: print("calibration FAILED (status %d) - retry with more rotation"%r.cal_status, flush=True); done=True
    elif t=='STATUSTEXT' and ('cal' in r.text.lower() or 'Compass' in r.text): print("  text:", r.text, flush=True)
