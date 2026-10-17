# First-flight review of a DataFlash .bin: python3 rt1176_bench_log_summary.py <file.bin>
# (MAVProxy: `log download latest` writes it; or copy it off the SD card.)
import sys, collections
from pymavlink import DFReader
f=sys.argv[1]; log=DFReader.DFReader_binary(f)
pm=[]; vibe=[]; att=[]; rcou=[]; arm=[]; msgs=[]
while True:
    m=log.recv_msg()
    if m is None: break
    t=m.get_type()
    if t=='PM': pm.append(m)
    elif t=='VIBE': vibe.append(m)
    elif t=='ATT': att.append(m)
    elif t=='RCOU': rcou.append(m)
    elif t=='ARM': arm.append(m)
    elif t=='MSG': msgs.append(m.Message)
def stat(xs): xs=list(xs); return (min(xs),sum(xs)/len(xs),max(xs)) if xs else (0,0,0)
print("file:",f); print("ARM events:",[(round(a.TimeUS/1e6,1),a.ArmState) for a in arm][:10])
if pm:
    nlon=[p.NLon for p in pm]; nloop=[p.NLoop for p in pm]; maxt=[p.MaxT for p in pm]; load=[getattr(p,'Load',0) for p in pm]
    print("PM: records=%d  loops/rec min/mean/max=%s  long-loops/rec=%s  MaxT us=%s  Load%%=%s"%(len(pm),tuple(round(x) for x in stat(nloop)),tuple(round(x,1) for x in stat(nlon)),tuple(round(x) for x in stat(maxt)),tuple(round(x/10,1) for x in stat(load))))
    if pm[0].TimeUS and len(pm)>1:
        dt=(pm[-1].TimeUS-pm[0].TimeUS)/1e6/(len(pm)-1); print("   implied loop rate: %.0f Hz"%(sum(nloop)/len(nloop)/dt))
if vibe: print("VIBE: X/Y/Z mean=%.1f/%.1f/%.1f  max=%.1f/%.1f/%.1f  clips=%d"%(stat(v.VibeX for v in vibe)[1],stat(v.VibeY for v in vibe)[1],stat(v.VibeZ for v in vibe)[1],stat(v.VibeX for v in vibe)[2],stat(v.VibeY for v in vibe)[2],stat(v.VibeZ for v in vibe)[2],sum(getattr(v,'Clip0',0)+getattr(v,'Clip1',0)+getattr(v,'Clip2',0) for v in vibe)))
if att:
    er=[abs(a.Roll-a.DesRoll) for a in att]; ep=[abs(a.Pitch-a.DesPitch) for a in att]
    print("ATT: n=%d  |roll err| mean/max=%.1f/%.1f deg  |pitch err| mean/max=%.1f/%.1f deg"%(len(att),stat(er)[1],stat(er)[2],stat(ep)[1],stat(ep)[2]))
if rcou:
    for ch in ('C1','C2','C3','C4'):
        v=[getattr(r,ch) for r in rcou]; print("RCOU %s: min/mean/max = %d/%d/%d us"%(ch,*(round(x) for x in stat(v))))
print("MSG lines:",len(msgs)); [print("  ",s) for s in msgs if any(k in s for k in ('stuck','Crash','EKF','Failsafe','Rate CPU','PreArm','Arm'))]
