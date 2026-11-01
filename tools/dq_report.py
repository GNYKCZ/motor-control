"""解析P1D0/P1D1停机后RAM块与P1T*原始诊断记录；不打开串口，不应用电零位或控制参数。"""
import binascii,json,re,sys
from pathlib import Path

HEADER='cfg ref count compute latency bench failed start sum_u sum_v zero pole_pairs fault reason'.split()
TRIP='reason sequence t u v angle detail'.split()
POINT='index sequence t reference id iq vd vq compute latency limited angle'.split()
TRACE_HEADER='version pre body total complete overflow frozen trip first freeze_reason freeze_sequence configs generation'.split()
TRACE_TIMER='stage generation read cr1 cr2 ccmr1 ccmr2 ccer bdtr'.split()
TRACE_TIMING='stage read psc arr rcr ccr1 ccr2 ccr3'.split()
TRACE_GPIO='stage read crh mapr idr moe preload'.split()
TRACE_SAMPLE='index sequence completed before after raw_u raw_v angle tim1 tim2 pins flags dir stage'.split()
TRACE_TAIL='actual total pre body complete overflow'.split()
TRACE_LAYOUT={'P1T0':TRACE_HEADER,'P1T1':TRACE_SAMPLE,'P1T2':TRACE_TIMER,
              'P1T3':TRACE_TIMING,'P1T4':TRACE_GPIO,'P1T5':TRACE_TAIL}
FLAG_VALID,FLAG_EDGE,FLAG_PRE,FLAG_POST=1,2,4,8
def signed(x):return x-0x100000000 if x&0x80000000 else x

def parse(raw):
    if not raw.endswith(b'\r\n') or not re.fullmatch(rb'P1(?:D[01]|E1),(?:[0-9A-F]{8},)+[0-9A-F]{4}\r\n',raw):
        raise ValueError('DQ frame format')
    parts=raw[:-2].split(b',');keys=HEADER if parts[0]==b'P1D0' else (TRIP if parts[0]==b'P1E1' else POINT)
    if len(parts)!=len(keys)+2 or binascii.crc_hqx(raw[:-6],0xffff)!=int(parts[-1],16):raise ValueError('DQ width/CRC')
    result=dict(zip(keys,[int(p,16) for p in parts[1:-1]]));result['format']=parts[0].decode()
    for key in (('ref',) if result['format']=='P1D0' else (('detail',) if result['format']=='P1E1' else ('reference','id','iq','vd','vq'))):
        result[key]=signed(result[key])
    if result['format']=='P1E1' and not (1<=result['reason']<=9 and result['u']<=4095 and result['v']<=4095 and result['angle']<=16383):raise ValueError('DQ trip fields')
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

def finish_block(header,points,trip):
    if header['fault'] or header['reason'] in (5,6):
        names={1:'raw_u',2:'raw_v',3:'inferred_w',4:'adc_endpoint',5:'sequence',6:'movement',7:'id',8:'iq',9:'q_tracking'}
        return dict(status='FAULT' if header['fault'] else 'STOPPED',accepted=False,configuration=header,
            first_trip=trip,first_trip_name=names.get(trip['reason']) if trip else 'unavailable',
            trace_records=len(points),trace_index_complete=len(points)==header['count'] and [p['index'] for p in points]==list(range(len(points))),
            calibration_verified=False,limits='Diagnostic fault block; does not pass250ms tracking acceptance')
    if header['cfg']==0x40002:
        if header['ref'] or header['zero']!=7453 or header['pole_pairs']!=7 or header['reason']!=4 or header['bench']!=32 or header['failed'] or header['compute']>200 or header['latency']:
            raise ValueError('Zero-vector diagnostic qualification')
        if not 15<=header['count']<=20 or len(points)!=header['count']:
            raise ValueError('Zero-vector observation count')
        for i,p in enumerate(points):
            if p['index']!=i or p['reference'] or p['vd'] or p['vq'] or p['latency'] or p['limited'] or p['compute']>200 or abs(p['id'])>80 or abs(p['iq'])>80 or p['angle']>16383:
                raise ValueError('Diagnostic is not a fixed zero-vector record')
            if i and ((p['sequence']-points[i-1]['sequence'])&0xffffffff!=1 or not 990<=(p['t']-points[i-1]['t'])&0xffffffff<=1010):
                raise ValueError('Diagnostic raw sequence gap')
        return dict(status='DIAGNOSTIC_COMPLETE',accepted=False,diagnostic_only=True,configuration=header,
            points=len(points),maximum_abs_id_mA=max(abs(p['id']) for p in points),maximum_abs_iq_mA=max(abs(p['iq']) for p in points),
            calibration_verified=False,limits='Fixed equal PWM, PI never applied; this is not closed-loop acceptance')
    return analyze(header,points)

def parse_trace(frame):
    """单帧解析：宽度、hex布局与CCITT-FALSE CRC；不猜测缺失字段。"""
    match=re.fullmatch(rb'P1T[0-5],(?:[0-9A-F]{8},)+[0-9A-F]{4}\r\n',frame)
    if not match:raise ValueError('trace frame format')
    parts=frame[:-2].split(b',');name=parts[0].decode();keys=TRACE_LAYOUT[name]
    if name=='P1T0' and int(parts[1],16)==2:keys=keys+['adc_phase_us']
    if len(parts)!=len(keys)+2:raise ValueError('trace width')
    if binascii.crc_hqx(frame[:-6],0xffff)!=int(parts[-1],16):raise ValueError('trace CRC')
    return {"format":name,**{key:int(value,16) for key,value in zip(keys,parts[1:-1])}}

def read_trace(frames):
    """按固定顺序验证一条原始记录块并返回判定。
    结构问题（缺帧、重复、错序、CRC、尾帧计数不符）抛异常且不补齐；
    物理不完整（恢复窗口未收满、溢出）只记录标志，由调用者决定是否接受。"""
    if not frames or frames[0]['format']!='P1T0':raise ValueError('trace missing header')
    header=frames[0];position=1
    if header['version']==2 and header.get('adc_phase_us') not in (225,250):raise ValueError('trace ADC phase')
    if header['version'] not in (1,2):raise ValueError('trace format version')
    configs=[]
    for _ in range(header['configs']):
        triple=[]
        for name in ('P1T2','P1T3','P1T4'):
            if position>=len(frames) or frames[position]['format']!=name:raise ValueError('trace config order')
            triple.append(frames[position]);position+=1
        if len({item['stage'] for item in triple})!=1 or triple[1]['read']!=triple[2]['read']:
            raise ValueError('trace config stage mismatch')
        configs.append(triple)
    stages=[item[0]['stage'] for item in configs]
    if stages!=sorted(stages) or len(set(stages))!=len(stages):raise ValueError('trace config duplicate stage')
    samples=[]
    for index in range(header['total']):
        if position>=len(frames) or frames[position]['format']!='P1T1':raise ValueError('trace sample order')
        sample=frames[position];position+=1
        if sample['index']!=index:raise ValueError('trace sample index')
        samples.append(sample)
    if position!=len(frames)-1 or frames[position]['format']!='P1T5':raise ValueError('trace missing tail')
    tail=frames[position]
    for key in ('total','pre','body'):
        if tail[key]!=header[key]:raise ValueError('trace tail count mismatch')
    if tail['actual']!=len(samples):raise ValueError('trace actual count mismatch')
    if tail['complete']!=header['complete'] or tail['overflow']!=header['overflow']:
        raise ValueError('trace completeness mismatch')
    if header['pre']+header['body']!=header['total']:raise ValueError('trace segment sum')
    for index,sample in enumerate(samples):
        if not 0<=sample['raw_u']<=4095 or not 0<=sample['raw_v']<=4095:raise ValueError('trace raw range')
        if sample['angle']>16383 or sample['tim1']>3600 or sample['tim2']>65535:raise ValueError('trace counter range')
        if sample['pins']>0x0f or sample['flags']>0x0f or sample['dir']>1 or sample['stage']>3:
            raise ValueError('trace field range')
        if not sample['completed']<=sample['before']<=sample['after']:raise ValueError('trace time order')
        if sample['after']-sample['before']>2 and not sample['flags']&FLAG_EDGE:
            raise ValueError('trace missing edge-uncertainty flag')
        in_pre=bool(sample['flags']&FLAG_PRE)
        if in_pre!=(index<header['pre']):raise ValueError('trace pre-action segment')
        if index and (samples[index-1]['flags']&FLAG_POST) and not sample['flags']&FLAG_POST:
            raise ValueError('trace post segment not contiguous')
        if sample['flags']&FLAG_POST and sample['stage']!=3:
            raise ValueError('trace recovery sample outside STOPPED stage')
        if index and not 0<(sample['sequence']-samples[index-1]['sequence'])&0xffffffff<0x80000000:
            raise ValueError('trace sequence not increasing')
        if index and not 0<(sample['completed']-samples[index-1]['completed'])&0xffffffff<=3000:
            raise ValueError('trace completion gap')
        if sample['stage']>=3 and (sample['pins']&0x8 or sample['pins']&0x7):
            raise ValueError('trace stopped sample still shows output')
    post=sum(1 for sample in samples if sample['flags']&FLAG_POST)
    if header['complete'] and post!=8:raise ValueError('trace complete without full recovery window')
    control=[item[0] for item in configs];timing=[item[1] for item in configs];gpio=[item[2] for item in configs]
    # IDLE在board_start_sampling之前记录，所以CEN=0是正确事实；只有动作阶段才要求计数器已运行。
    for entry in control:
        if entry['stage']<=2 and entry['cr1']&0x60!=0x60:raise ValueError('trace centre-aligned mode')
        if entry['stage']==0 and entry['cr1']&1:raise ValueError('trace IDLE recorded after counter start')
        if entry['stage'] in (1,2) and not entry['cr1']&1:raise ValueError('trace action without counter running')
        if entry['stage']<=2 and (entry['ccmr1']&0x808!=0x808 or not entry['ccmr2']&0x8):
            raise ValueError('trace preload enable')
        if entry['stage'] in (1,2) and not entry['bdtr']&0x8000:raise ValueError('trace MOE missing in action')
        if entry['stage'] in (0,3) and entry['bdtr']&0x8000:raise ValueError('trace MOE outside action')
    for entry in timing:
        # 启动相位必须与验收几何一致；停机阶段只记录实际值，被破坏的配置是证据而不是格式错误。
        if entry['stage']<=2 and (entry['psc'] or entry['arr']!=1800 or entry['rcr']!=39):
            raise ValueError('trace timer setup')
        if entry['stage'] in (1,2) and (entry['ccr1'],entry['ccr2'],entry['ccr3'])!=(900,900,900):
            raise ValueError('trace fixed fifty-percent compare')
    for entry in gpio:
        if entry['stage'] in (1,2) and (entry['crh']&0xfff)!=0xaaa:raise ValueError('trace AF pins in action')
        if entry['stage'] in (0,3) and (entry['crh']&0xfff)!=0x222:raise ValueError('trace idle pins')
        if entry['stage']==1 and not entry['preload']:raise ValueError('trace stage1 must be preload only')
        if entry['stage']==2 and entry['preload']:raise ValueError('trace stage2 already effective')
    if header['frozen'] and header['trip']!=0xffffffff and not 0<=header['trip']<header['total']:
        raise ValueError('trace trip index outside block')
    # IRQ快照应落在采样触发之后；仅在距前/后切换沿有余量时用于核对相位。
    phase=header.get('adc_phase_us')
    usable=[s for s in samples if s['stage']==2 and s['flags']&FLAG_VALID and not s['flags']&FLAG_EDGE]
    consistent=None
    if phase and usable:
        consistent=all(phase<=s['tim2']<phase+12 for s in usable)
    return dict(phase_consistent=consistent,phase_samples=len(usable),header=header,configs=configs,samples=samples,tail=tail,post=post,
        complete=bool(header['complete']),overflow=bool(header['overflow']),frozen=bool(header['frozen']),
        trip_index=header['trip'],stages=stages)

def trace_reports(raw):
    """从一段串口导出里切出所有P1T*块；同一块内出现重复头帧即拒绝。"""
    blocks=[];current=[]
    for match in re.finditer(rb'P1T[0-5],[0-9A-F,]+\r?\n',raw):
        frame=match.group(0)
        if not frame.endswith(b'\r\n'):frame=frame[:-1]+b'\r\n' # 文本模式读取会折叠CRLF
        record=parse_trace(frame)
        if record['format']=='P1T0':
            if current:blocks.append(read_trace(current))
            current=[record]
        elif current:current.append(record)
        else:raise ValueError('trace sample without header')
    if current:blocks.append(read_trace(current))
    if not blocks:raise ValueError('trace no block')
    return blocks

def reports(raw):
    groups=[];header=None;points=[];trip=None
    for frame in re.findall(rb'P1(?:D[01]|E1),[0-9A-F,]+\r?\n',raw):
        if not frame.endswith(b'\r\n'):frame=frame[:-1]+b'\r\n'
        record=parse(frame)
        if record['format']=='P1D0':
            if header is not None:groups.append(finish_block(header,points,trip))
            header=record if record['count'] or record['fault'] or record['reason'] in (5,6) else None
            points=[];trip=None
        elif header is not None:
            if record['format']=='P1E1':
                if trip is not None:raise ValueError('DQ duplicate first-trip record')
                trip=record
            else:points.append(record)
    if header is not None:groups.append(finish_block(header,points,trip))
    if not groups:raise ValueError('DQ no action block')
    return groups
if __name__=='__main__':print(json.dumps(reports(Path(sys.argv[1]).read_bytes()),ensure_ascii=False,indent=2))
