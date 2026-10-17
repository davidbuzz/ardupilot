# Motors 2,3,4 at 1200 us for 4 s each, 3 s apart, capturing the FlexPWM dump and
# RCOUT lines (instrumented build) around each test. PROPS OFF.
import time, glob, struct, re
from pymavlink import mavutil
mav=mavutil.mavlink
p=glob.glob('/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*-if-mavlink')[0]
m=mavutil.mavlink_connection(p,baud=115200); hb=m.wait_heartbeat(timeout=30); ts=hb.get_srcSystem()
ser=m.port; ser.timeout=0; parser=mav.MAVLink(None); parser.robust_parsing=True
def capture(secs, tag):
    t0=time.time(); text=bytearray(); out=[]
    while time.time()-t0<secs:
        d=ser.read(65536)
        if not d: time.sleep(0.002); continue
        i=0
        while i<len(d):
            b=d[i]
            if b==0xFD and i+1<len(d):
                ln=d[i+1]; fl=12+ln+(13 if (i+2<len(d) and d[i+2]&1) else 0)
                for mm in (parser.parse_buffer(d[i:i+fl]) or []):
                    if mm.get_type()=='STATUSTEXT' and (mm.text.startswith(('m1 ','m2 ','m3 ','m4 ','RCOUT','FPWM','starting','finished')) or 'otor' in mm.text): out.append("  [%s +%.1fs] %s"%(tag,time.time()-t0,mm.text))
                    elif mm.get_type()=='COMMAND_ACK' and mm.command==mav.MAV_CMD_DO_MOTOR_TEST: out.append("  [%s +%.1fs] ACK result %d"%(tag,time.time()-t0,mm.result))
                i+=fl; continue
            if 32<=b<127 or b==10: text.append(b)
            if b==10:
                s=text.decode(errors='ignore').strip(); text=bytearray()
                if re.match(r'^(m[1-4] |FPWM)',s): out.append("  [%s +%.1fs] printk: %s"%(tag,time.time()-t0,s[:120]))
            i+=1
    return out
for motor in (2,3,4):
    print("=== motor %d: 1200 us for 4 s ==="%motor, flush=True)
    m.mav.command_long_send(ts,1,mav.MAV_CMD_DO_MOTOR_TEST,0,motor,1,1200,4,1,0,0)
    for l in capture(5.0,"m%d"%motor): print(l, flush=True)
    time.sleep(3)
print("done: which of motors 2, 3, 4 turned?")
