"""解析P1D0/P1D1停机后RAM块；不打开串口，不应用电零位或控制参数。"""
import binascii,json,re,sys
from pathlib import Path

HEADER='cfg ref count compute latency bench failed start sum_u sum_v zero pole_pairs fault reason'.split()
POINT='index sequence t reference id iq vd vq compute latency limited angle'.split()
def signed(x):return x-0x100000000 if x&0x80000000 else x

def parse(raw):
    if not raw.endswith(b'\r\n') or not re.fullmatch(rb'P1D[01],(?:[0-9A-F]{8},)+[0-9A-F]{4}\r\n',raw):
        raise ValueError('DQ frame format')
    parts=raw[:-2].split(b',');keys=HEADER if parts[0]==b'P1D0' else POINT
    if len(parts)!=len(keys)+2 or binascii.crc_hqx(raw[:-6],0xffff)!=int(parts[-1],16):raise ValueError('DQ width/CRC')
    result=dict(zip(keys,[int(p,16) for p in parts[1:-1]]));result['format']=parts[0].decode()
    for key in ('ref',) if result['format']=='P1D0' else ('reference','id','iq','vd','vq'):
        result[key]=signed(result[key])
    return result

def analyze(header,points):
    if header['cfg']!=0x40001 or header['ref'] not in (0,40,-40) or header['zero']!=7453 or header['pole_pairs']!=7:
        raise ValueError('DQ configuration identity')
    if header['fault'] or header['reason']!=4 or header['failed'] or header['bench']!=32 or header['compute']>200:
        raise ValueError('DQ fault/incomplete timing qualification')
    if not 245<=header['count']<=256 or len(points)!=header['count']:raise ValueError('DQ incomplete transient')
    plateau=[]
    for i,p in enumerate(points):
        if p['index']!=i or p['compute']>200 or not 740<=p['latency']<=810 or p['limited'] not in (0,1) or p['angle']>16383:
            raise ValueError('DQ record ordering/timing/range')
        if i and ((p['sequence']-points[i-1]['sequence'])&0xffffffff!=1 or not 990<=(p['t']-points[i-1]['t'])&0xffffffff<=1010):
            raise ValueError('DQ dropped sample')
        elapsed=(p['t']-header['start'])&0xffffffff
        if not 0<elapsed<250000 or (i==0 and elapsed>500) or abs(p['reference'])>40 or abs(p['id'])>80 or abs(p['iq'])>80 or p['vd']**2+p['vq']**2>401**2:
            raise ValueError('DQ stimulus/time envelope')
        if 180000<=elapsed<225000:plateau.append(abs(p['reference']-p['iq']))
    if not plateau:raise ValueError('DQ missing plateau')
    return dict(configuration=header,points=len(points),plateau_max_q_error_mA=max(plateau),
        maximum_abs_id_mA=max(abs(p['id']) for p in points),peak_measured_iq_mA=max(abs(p['iq']) for p in points),
        maximum_observed_compute_us=max(p['compute'] for p in points),maximum_sample_to_apply_us=max(p['latency'] for p in points),
        voltage_limited_samples=sum(p['limited'] for p in points),last_reference_mA=points[-1]['reference'],last_measured_iq_mA=points[-1]['iq'],
        calibration_verified=False,absolute_current_verified=False,electrical_zero_candidate_used_in_RAM=True,parameters_persisted=False,
        limits='1kHz nominal currents, p7/d+1/candidate zero; tracking does not prove zero or phase order; PWM is stopped before exporting')

def reports(raw):
    groups=[];header=None;points=[]
    for frame in re.findall(rb'P1D[01],[0-9A-F,]+\r?\n',raw):
        if not frame.endswith(b'\r\n'):frame=frame[:-1]+b'\r\n'
        record=parse(frame)
        if record['format']=='P1D0':
            if header is not None:groups.append(analyze(header,points))
            header=record if record['count'] else None;points=[]
        elif header is not None:points.append(record)
    if header is not None:groups.append(analyze(header,points))
    if not groups:raise ValueError('DQ no completed action block')
    return groups
if __name__=='__main__':print(json.dumps(reports(Path(sys.argv[1]).read_bytes()),ensure_ascii=False,indent=2))
