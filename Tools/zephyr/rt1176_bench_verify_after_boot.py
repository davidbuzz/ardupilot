# After flash + reset + reboot: is the starvation gone?  60 s of evidence on 14550.
import time, sys
from pymavlink import mavutil
link=sys.argv[1] if len(sys.argv)>1 else 'udpin:127.0.0.1:14550'
m=mavutil.mavlink_connection(link); hb=m.wait_heartbeat(timeout=120)
if not hb: print("NO HEARTBEAT"); sys.exit(2)
ts,tc=hb.get_srcSystem(),1; t0=time.time(); txt=[]; loads=[]
while time.time()-t0<60:
    r=m.recv_match(type=['STATUSTEXT','SYS_STATUS'],blocking=True,timeout=1)
    if r is None: continue
    if r.get_type()=='STATUSTEXT': txt.append("+%2.0fs %d:%s"%(time.time()-t0,r.severity,r.text))
    else: loads.append(r.load/10.0)
print("SYS_STATUS load %%: n=%d min=%.0f mean=%.0f max=%.0f"%(len(loads),min(loads or [0]),sum(loads)/max(1,len(loads)),max(loads or [0])))
for l in txt: print(l)
# the definitive check: a single PARAM_REQUEST_READ answered promptly means the IO thread runs
m.mav.param_request_read_send(ts,tc,b'SCHED_LOOP_RATE',-1); t1=time.time(); v=None
while time.time()-t1<10:
    r=m.recv_match(type='PARAM_VALUE',blocking=True,timeout=1)
    if r and r.param_id.rstrip('\x00')=='SCHED_LOOP_RATE': v=r.param_value; break
print("PARAM_REQUEST_READ SCHED_LOOP_RATE -> %s in %.1fs"%(v,time.time()-t1))
