# USB link A/B: throughput with every stream at 50 Hz (x2), FTP open latency quiet
# vs streaming (x2 each), and a full parameter fetch time. Sole listener on serial.
import time, glob, struct
from pymavlink import mavutil
mav=mavutil.mavlink
p=glob.glob('/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*-if-mavlink')[0]
m=mavutil.mavlink_connection(p,baud=115200); hb=m.wait_heartbeat(timeout=30); ts=hb.get_srcSystem(); ser=m.port
def streams(on):
    for sid in (0,1,3,6,10,11,12): m.mav.request_data_stream_send(ts,1,sid,50 if on else 0,1 if on else 0)
def throughput(secs=5):
    ser.timeout=0; ser.read(65536); t0=time.perf_counter(); tot=0; reads=[]
    while time.perf_counter()-t0<secs:
        d=ser.read(65536)
        if d: tot+=len(d); reads.append(len(d))
        else: time.sleep(0.0005)
    reads.sort(); return tot/secs, (reads[len(reads)//2] if reads else 0), (reads[-1] if reads else 0)
seq=200
def ftp_open(wait=3.0):
    global seq
    for op,path in ((2,b''),(4,b'@SYS/tasks.txt')):
        pl=struct.pack('<HBBBBBBI',seq,0,op,len(path),0,0,0,0)+path; pl+=b'\x00'*(251-len(pl)); m.mav.file_transfer_protocol_send(0,ts,1,list(pl)); want=seq+1; seq+=1
    t0=time.time()
    while time.time()-t0<wait:
        r=m.recv_match(type='FILE_TRANSFER_PROTOCOL',blocking=True,timeout=0.5)
        if r and struct.unpack('<H',bytes(r.payload)[:2])[0]==want: return time.time()-t0
    return None
for i in (1,2):
    streams(True); time.sleep(1); bps,med,mx=throughput(); streams(False)
    print("throughput run %d: %.0f B/s  (median read %d B, max %d B)"%(i,bps,med,mx), flush=True); time.sleep(1); ser.read(65536)
for label,on in (("quiet",False),("streams@50Hz",True),("quiet",False),("streams@50Hz",True)):
    streams(on); time.sleep(1.5); res=[]
    for k in range(3):
        t=ftp_open(); res.append("%.2fs"%t if t is not None else "none"); time.sleep(0.3)
    print("FTP open %-13s %s"%(label,res), flush=True)
streams(False); time.sleep(1)
got=set(); t0=time.time(); m.mav.param_request_list_send(ts,1); count=None
while time.time()-t0<120:
    r=m.recv_match(type='PARAM_VALUE',blocking=True,timeout=3)
    if r is None: break
    count=r.param_count; got.add(r.param_index)
    if len(got)>=count: break
print("param fetch: %d of %s in %.1f s"%(len(got),count,time.time()-t0))
