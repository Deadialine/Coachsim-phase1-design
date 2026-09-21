"""Packet accounting test, independent of the raw-signal generator."""
import json
from pathlib import Path

def simulate(seconds=600,drop_every=0):
    # One 10 ms packet per stream: 20 EMG rows or one IMU row.
    report={'provenance':'synthetic','duration_seconds':seconds,'physical_transport_measured':False,'streams':{}}
    for stream,rows in [('emg',20),('imu',1)]:
        sent=list(range(seconds*100))
        received=[i for i in sent if not drop_every or (i+1)%drop_every]
        unique=set(received);missing=len(sent)-len(unique)
        report['streams'][stream]={'packets_sent':len(sent),'packets_received':len(received),'duplicates':len(received)-len(unique),'reordered':sum(b<a for a,b in zip(received,received[1:])),'packets_missing':missing,'packet_loss_fraction':missing/len(sent),'sample_slots_missing':missing*rows,'pass_below_0_5pct':missing/len(sent)<.005}
    return report

if __name__=='__main__':
    nominal=simulate();negative=simulate(drop_every=100)
    assert all(v['pass_below_0_5pct'] for v in nominal['streams'].values())
    assert all(not v['pass_below_0_5pct'] for v in negative['streams'].values())
    path=Path(__file__).resolve().parents[1]/'evidence/reports/transport-simulation.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps({'nominal':nominal,'negative_test':negative},indent=2)+'\n',encoding='utf-8')
    print('Packet-accounting simulation passed nominal and 1% loss negative checks.')
