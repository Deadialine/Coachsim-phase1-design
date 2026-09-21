"""Reproducible synthetic bench evidence. Python 3.10+, standard library only.

Generate CSV streams on disk, read them back independently, then ZIP and hash.
Never represents a sensor, physiological recording, or measured transport latency.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import zipfile

CLASSES = ['neutral_rest','wrist_flexion','wrist_extension','radial_deviation','ulnar_deviation','forearm_pronation','forearm_supination']
CONFIG = dict(id='coachsim-sim-v2.0',emg_hz=2000,imu_hz=100,channels=4,adc_min=0,adc_max=4095,window_us=200000,hop_us=50000,confidence_min=.7,stability_us=250000)

def write_csv(path, header, rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f,lineterminator='\n'); w.writerow(header); w.writerows(rows)

def emg_rows(seconds, channels=4, faults=False):
    for i in range(seconds*2000):
        if faults and i%100==50: continue
        t=i/2000; moving=t>=seconds/2; a=180 if moving else 12
        values=[round(2048+a*(1+c*.1)*math.sin(2*math.pi*(67+c*13)*t+c)+a*.33*math.sin(2*math.pi*(173+c*7)*t)) for c in range(1,channels+1)]
        clip=faults and i%100<2
        if clip: values[0]=4095
        # The rate-error scenario also has a slower device clock.
        yield [i,round(i*500*(1.02 if faults else 1)),*values,int(clip)]

def imu_rows(seconds, faults=False):
    for i in range(seconds*100):
        if faults and i%100==50: continue
        t=i/100; a=.15 if t>=seconds/2 else .005
        yield [i,i*10000,a*math.sin(t*2),a*math.cos(t*2),1+a*.2*math.sin(t),a*30*math.cos(t*2),a*20*math.sin(t*2),a*10*math.cos(t),0]

def inspect_stream(path,hz,seconds,channels=0):
    count=0; last_i=last_t=None; missing_internal=0; nonmonotonic=0; invalid=0; flagged=0
    min_dt=math.inf; max_dt=0; adjusted=[]; sums=[0.]*channels; sums2=[0.]*channels; clipped=[0]*channels
    segment_count=[0,0]; seg_sums=[[0.]*channels for _ in range(2)]; seg_sums2=[[0.]*channels for _ in range(2)]
    first_i=first_t=0
    with path.open(newline='',encoding='utf-8') as f:
        for row in csv.DictReader(f):
            idx=int(row['sample_index']); t=int(row['t_us'])
            if count==0: first_i,first_t=idx,t
            if last_i is not None:
                di=idx-last_i;dt=t-last_t
                nonmonotonic+=int(di<=0 or dt<=0);missing_internal+=max(0,di-1)
                min_dt=min(min_dt,dt);max_dt=max(max_dt,dt)
                if di>0 and len(adjusted)<100000: adjusted.append(dt/di)
            invalid+=int(t<0 or t>=seconds*1e6 or idx<0)
            last_i,last_t=idx,t; count+=1
            flagged+=int(int(row['adc_flags' if channels else 'sensor_flags'])!=0)
            if not channels: invalid+=sum(not math.isfinite(float(row[k])) for k in ['ax','ay','az','gx','gy','gz'])
            seg=int(idx>=seconds*hz/2);segment_count[seg]+=1
            for c in range(channels):
                v=float(row[f'emg_ch{c+1}']); invalid+=int(not math.isfinite(v) or not 0<=v<=4095)
                clipped[c]+=int(v<=0 or v>=4095); sums[c]+=v;sums2[c]+=v*v
                seg_sums[seg][c]+=v;seg_sums2[seg][c]+=v*v
    expected=seconds*hz; missing=max(0,expected-count);rate=count/seconds
    clock_rate=((last_i-first_i)*1e6/(last_t-first_t)) if count>1 and last_t>first_t else 0
    clip_fractions=[x/count if count else 0 for x in clipped]
    def ac_rms(s,s2,n): return math.sqrt(max(0,s2/n-(s/n)**2)) if n else None
    signal=[]
    for c in range(channels):
        rest=ac_rms(seg_sums[0][c],seg_sums2[0][c],segment_count[0]);motion=ac_rms(seg_sums[1][c],seg_sums2[1][c],segment_count[1])
        signal.append(dict(channel=c+1,mean_adc=sums[c]/count,ac_rms_adc=ac_rms(sums[c],sums2[c],count),still_ac_rms_adc=rest,motion_ac_rms_adc=motion,motion_to_still_db=20*math.log10(motion/rest) if rest and motion else None,clipped_fraction=clip_fractions[c]))
    checks=dict(nonempty=count>1,time_and_sequence_monotonic=nonmonotonic==0,valid_values_and_bounds=invalid==0,received_rate_within_1pct=abs(rate/hz-1)<=.01,clock_rate_within_1pct=abs(clock_rate/hz-1)<=.01,missing_below_0_5pct=missing/expected<.005,clipping_each_channel_below_1pct=all(v<.01 for v in clip_fractions))
    return dict(samples=count,expected_samples=expected,missing_samples=missing,missing_fraction=missing/expected,internal_sequence_gaps=missing_internal,first_index=first_i,last_index=last_i,first_t_us=first_t,last_t_us=last_t,received_hz=rate,clock_hz=clock_rate,min_dt_us=min_dt if count>1 else None,max_dt_us=max_dt if count>1 else None,normalized_period_median_us=statistics.median(adjusted) if adjusted else None,nonmonotonic=nonmonotonic,invalid=invalid,flagged_rows=flagged,channels=signal,checks=checks,pass_all=all(checks.values()))

def generate(out,seconds,channels,faults=False):
    if out.exists(): raise ValueError(f'Refusing to overwrite existing evidence: {out}')
    out.mkdir(parents=True)
    meta=dict(schema_version=2,participant_id='SIM-BENCH',session_id=out.name,t0_unix=0,time_origin='session_start',provenance='synthetic',device_version='virtual-node-2.0',firmware_version='not-hardware',model_version='synthetic-overlay-1',config={**CONFIG,'channels':channels},channel_map=[dict(id=f'emg_ch{c}',unit='adc_count',placement='simulated',calibrated=False) for c in range(1,channels+1)],placement_metadata=dict(mode='simulation'),duration_us=seconds*1000000,generator='bench_v2.py v2.0',fault_injection=faults,clock_note='Virtual timeline; no RTC, wall-clock, packet transport, or human measurements.')
    (out/'session.json').write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
    write_csv(out/'emg.csv',['sample_index','t_us',*[f'emg_ch{c}' for c in range(1,channels+1)],'adc_flags'],emg_rows(seconds,channels,faults))
    write_csv(out/'imu.csv',['sample_index','t_us','ax','ay','az','gx','gy','gz','sensor_flags'],imu_rows(seconds,faults))
    write_csv(out/'events.csv',['t_us','event_type','target_posture','repetition','block'],[[0,'rest','neutral_rest',0,0],[seconds*500000,'cue','wrist_flexion',1,1],[seconds*1000000,'complete','neutral_rest',1,1]])
    write_csv(out/'predictions.csv',['t_us','predicted_posture','confidence','model_version','latency_ms','quality_ok'],([i*50000,'uncertain' if faults else ('neutral_rest' if i*50000<seconds*500000 else 'wrist_flexion'),.2 if faults else .87,'synthetic-overlay-1',25,0 if faults else 1] for i in range(seconds*20)))
    report=dict(provenance='synthetic',duration_seconds=seconds,channel_count=channels,hardware_verified=False,transport_packet_loss='not measured; sample gaps assessed separately',latency='25 ms injected test value; end-to-end latency not measured',emg=inspect_stream(out/'emg.csv',2000,seconds,channels),imu=inspect_stream(out/'imu.csv',100,seconds))
    report['pass_all']=report['emg']['pass_all'] and report['imu']['pass_all']
    report['files']={p.name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(out.iterdir()) if p.is_file()}
    archive=out.with_suffix('.zip')
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(out.iterdir()):
            info=zipfile.ZipInfo(p.name,date_time=(2026,9,21,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED;z.writestr(info,p.read_bytes())
    report['archive']=dict(name=archive.name,bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    out.with_suffix('.report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(path=str(archive),samples=report['emg']['samples'],pass_all=report['pass_all'],expected_failure=faults)))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=int,default=600);p.add_argument('--channels',type=int,choices=[1,4],default=4);p.add_argument('--faults',action='store_true');args=p.parse_args()
    if args.seconds<2: p.error('Use at least two seconds for still/motion segments')
    r=generate(args.out,args.seconds,args.channels,args.faults)
    raise SystemExit(0 if r['pass_all'] != args.faults else 1)
