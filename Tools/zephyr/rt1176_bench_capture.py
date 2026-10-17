# Raw console capture on the USB CDC port: MAVLink frames AND printk text share the
# link. Splits them, prints the diagnostic lines, optionally runs tests mid-capture.
#   python3 rt1176_bench_capture.py <seconds> [motortest] [ftpload]
import sys, time, glob, struct, serial, re
from pymavlink import mavutil
mav=mavutil.mavlink
secs=int(sys.argv[1]) if len(sys.argv)>1 else 60; opts=sys.argv[2:]
p=glob.glob('/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*-if-mavlink')[0]
m=mavutil.mavlink_connection(p,baud=115200); hb=m.wait_heartbeat(timeout=30); ts=hb.get_srcSystem()
ser=m.port; ser.timeout=0
parser=mav.MAVLink(None); parser.robust_parsing=True
text=bytearray(); texts=[]; stx=[]; t0=time.time(); ftp_seq=100
def ftp_open(path=b'@SYS/tasks.txt'):
    global ftp_seq
    pl=struct.pack('<HBBBBBBI',ftp_seq,0,2,0,0,0,0,0); pl+=b'\x00'*(251-len(pl)); m.mav.file_transfer_protocol_send(0,ts,1,list(pl)); ftp_seq+=1
    pl=struct.pack('<HBBBBBBI',ftp_seq,0,4,len(path),0,0,0,0)+path; pl+=b'\x00'*(251-len(pl)); m.mav.file_transfer_protocol_send(0,ts,1,list(pl)); ftp_seq+=1
did=set()
while time.time()-t0<secs:
    el=time.time()-t0
    if 'motortest' in opts and el>15 and 'mt' not in did:
        did.add('mt'); m.mav.command_long_send(ts,1,mav.MAV_CMD_DO_MOTOR_TEST,0,1,1,1200,4,1,0,0); texts.append((el,"[test] DO_MOTOR_TEST motor1 1200us 4s sent"))
    if 'ftpload' in opts:
        if el>25 and 'f1' not in did: did.add('f1'); ftp_open(); texts.append((el,"[test] FTP open (quiet)"))
        if el>35 and 's1' not in did:
            did.add('s1'); [m.mav.request_data_stream_send(ts,1,sid,50,1) for sid in (0,1,3,6,10,11,12)]; texts.append((el,"[test] streams ON @50Hz"))
        if el>40 and 'f2' not in did: did.add('f2'); ftp_open(); texts.append((el,"[test] FTP open (streams on)"))
        if el>50 and 's0' not in did:
            did.add('s0'); [m.mav.request_data_stream_send(ts,1,sid,0,0) for sid in (0,1,3,6,10,11,12)]; texts.append((el,"[test] streams OFF"))
    d=ser.read(65536)
    if not d: time.sleep(0.002); continue
    i=0
    while i<len(d):
        b=d[i]
        if b==0xFD and i+1<len(d):
            ln=d[i+1]; fl=12+ln+(13 if (i+2<len(d) and d[i+2]&1) else 0)
            frame=d[i:i+fl]
            for mm in (parser.parse_buffer(frame) or []):
                if mm.get_type()=='STATUSTEXT': stx.append((time.time()-t0,mm.severity,mm.text))
                elif mm.get_type()=='FILE_TRANSFER_PROTOCOL':
                    pb=bytes(mm.payload); sq,ss,op=struct.unpack('<HBB',pb[:4]); texts.append((time.time()-t0,"[ftp reply] seq=%d %s"%(sq,'ACK' if op==128 else 'NAK%d'%pb[12])))
            i+=fl; continue
        if 32<=b<127 or b in (10,13): text.append(b)
        if b==10:
            s=text.decode(errors='ignore').strip(); text=bytearray()
            if len(s)>=8 and re.match(r'^[A-Za-z]',s): texts.append((time.time()-t0,s))
        i+=1
print("=== STATUSTEXT (diagnostic lines) ===")
for t,sev,s in stx:
    if not s.startswith(('PreArm','CrashDump')) or s.startswith('MB '): print("+%5.1fs [%d] %s"%(t,sev,s))
print("=== console printk / test markers ===")
for t,s in texts:
    if any(k in s for k in ('MICROBENCH','MB ','CPU','DELAYPROF','UARTSTAT','BUSCB','I2CERR','CPUBAND','FTPTHREAD','LOOPRATE','FPWM','FTPDIAG','[test]','[ftp')): print("+%5.1fs %s"%(t,s[:160]))
