"""真实前台电流环/寄存器/生命周期集成；只运行主机，不访问串口或硬件。"""
from pathlib import Path
from dq_report import reports,parse,parse_trace,trace_reports
from sensor_capture import parse_line
import subprocess,os,json,hashlib,re,binascii
r=Path(__file__).resolve().parents[1];fw=r/'firmware/stm32f103';out=r/'build/dq_current/host';out.mkdir(parents=True,exist_ok=True)
gcc=Path('D:/mingw64/bin/gcc.exe');env=os.environ.copy();env['PATH']=str(gcc.parent)+os.pathsep+env.get('PATH','')
model=r/'tests/m1/pwm_model';inc=[flag for p in (out,model,fw/'board',fw/'app',r/'control_core') for flag in ('-I',str(p))]
flags=['-O2','-g','-Wall','-Wextra','-Werror','-pedantic',*inc,'-DBOARD_DQ_CURRENT_LOOP=1','-DBOARD_ALIGNMENT_MODE=2']
sources=[fw/p for p in ('app/motor_runtime.c','app/command.c','app/current_loop.c','app/diag_trace.c','board/sample_frame.c')]+[r/'control_core'/p for p in ('feedback.c','sensor_input.c','current_control.c','foc_math.c')]
def loop_header(source):
 body=source.split('/* USER CODE BEGIN 3 */',1)[1].split('/* USER CODE END 3 */',1)[0]
 body=body.rsplit('}',1)[0] # USER3包含while的右括号，提取单轮主体。
 return 'static void production_poll_iteration(void) {\n'+body+'\n}\n'
(out/'dq_production_loop.h').write_text(loop_header((fw/'Core/Src/main.c').read_text(encoding='utf-8')),encoding='utf-8')
old_main=subprocess.check_output(['git','show','ca5bf7a206c63e75685a6624cec80428922412f9:firmware/stm32f103/Core/Src/main.c'],cwd=r,text=True)
(out/'dq_production_loop_old.h').write_text(loop_header(old_main),encoding='utf-8')
steps=[]
inputs=[*sources,fw/'board/board_pwm.c',fw/'Core/Src/main.c',fw/'Core/Src/tim.c',fw/'m1_sampling.ioc',fw/'project_sources.json',r/'docs/dq_current_loop.md',r/'tests/m1/test_pwm_gate.cpp',r/'tests/m1/test_dq_current.cpp',r/'tests/control_core/test_current_control_bounded.c',Path(__file__),r/'tools/dq_report.py',r/'tools/sensor_capture.py']
inputs+=list((fw/'app').glob('*.h'))+list((fw/'board').glob('*.h'))+list((r/'control_core').glob('*.h'))+list(model.glob('*.h'))
hashes={p.relative_to(r).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=r,text=True).strip()
evidence=dict(head=head,input_sha256=hashes,result='IN PROGRESS',hardware='NOT RUN',steps=steps)
previous=out/'evidence.json'
if previous.exists():
 old=json.loads(previous.read_text())
 if old.get('result')!='PASS':
  from datetime import datetime
  (out/('failed_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')).write_bytes(previous.read_bytes())

def run(cmd):
 p=subprocess.run(list(map(str,cmd)),cwd=r,env=env,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=90);steps.append(dict(command=list(map(str,cmd)),returncode=p.returncode,output=p.stdout+p.stderr));evidence['result']='FAIL' if p.returncode else 'IN PROGRESS';previous.write_text(json.dumps(evidence,indent=2)+'\n')
 if p.returncode:print((p.stdout+p.stderr)[-2000:]);raise RuntimeError('DQ integration failed')
 return p.stdout
oracle=out/'bounded_oracle.exe'
run([gcc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic','-I',r/'control_core',r/'control_core/current_control.c',r/'control_core/foc_math.c',r/'tests/control_core/test_current_control_bounded.c','-lm','-o',oracle])
print(run([oracle]).strip())
assert parse_line(b"P1F1,FAILED_CONTROL_TIMING\r\n")["hint"]=="FAILED_CONTROL_TIMING"
for reference,vector in ((0,0),(40,0),(-40,0),(0,1)):
 tag=str(reference)+('_vector' if vector else '')
 define=['-DBOARD_DQ_VECTOR_DIAGNOSTIC='+str(vector),'-DBOARD_DQ_REFERENCE_MA='+str(reference)];objects=[]
 for p in sources:
  obj=out/(tag+'_'+p.stem+'.o');run([gcc,'-std=c11',*flags,*define,'-c',p,'-o',obj]);objects.append(obj)
 board=out/(tag+'_board.o');cpp=[gcc.parent/'g++.exe','-std=c++11',*flags,*define]
 run([*cpp,'-include',model/'linkage.h','-x','c++','-c',fw/'board/board_pwm.c','-o',board])
 exe=out/(tag+'.exe');run([*cpp,r/'tests/m1/test_dq_current.cpp',board,*objects,'-Wl,--wrap=motor_runtime_poll_feedback','-lm','-o',exe])
 text=run([exe]);full_output=steps[-1]['output'];failure=full_output.split('DQ_FAILURE_CAPTURE_BEGIN\n',1)[1].split('DQ_FAILURE_CAPTURE_END',1)[0]
 failure_records=[parse((line+'\r\n').encode()) for line in failure.splitlines() if line.startswith(('P1D0,','P1D1,','P1E1,'))]
 trip=next(x for x in failure_records if x['format']=='P1E1')
 assert trip['reason']==1 and trip['u']==2157 and trip['v']==2028 and trip['detail']==130*512,trip
 assert sum(x['format']=='P1D0' for x in failure_records)==1 and sum(x['format']=='P1D1' for x in failure_records)==4
 failure_report=reports(failure.encode())[0];assert failure_report['accepted'] is False and failure_report['first_trip_name']=='raw_u' and failure_report['trace_index_complete']
 (out/(tag+'_trace.txt')).write_text(text,encoding='utf-8');parsed=reports(text.encode());assert len(parsed)==4
 for report in parsed:
  assert (report['status']=='DIAGNOSTIC_COMPLETE' and report['accepted'] is False) if vector else (report['plateau_max_q_error_mA']<=6 and report['maximum_abs_id_mA']<=2),report
  assert report['configuration']['ref']==reference
 frame=next(line+'\r\n' for line in text.splitlines() if line.startswith('P1D1,'))
 damaged=bytearray(frame.encode());damaged[8]=ord('F') if damaged[8]!=ord('F') else ord('0')
 try:parse(bytes(damaged));raise AssertionError('corruption accepted')
 except ValueError:pass
 if vector:
  observer=full_output.split('DQ_VECTOR_OBSERVER_BEGIN\n',1)[1].split('DQ_VECTOR_OBSERVER_END',1)[0]
  observer_records=[parse((line+'\r\n').encode()) for line in observer.splitlines() if line.startswith('P1D1,')]
  assert len(observer_records)==4 and any(p['id'] or p['iq'] for p in observer_records)
  assert all(not p['vd'] and not p['vq'] and not p['latency'] for p in observer_records)
  # 原始诊断记录：五条终止路径各导出一次，顺序/CRC/计数由独立解析器判定。
  blocks={}
  for match in re.finditer(r'DQ_TRACE_(\w+)_BEGIN\n(.*?)DQ_TRACE_\1_END',full_output,re.S):
   tag=match.group(1);assert tag not in blocks,'duplicate trace block '+tag
   frames=[parse_trace((line+'\r\n').encode()) for line in match.group(2).splitlines() if line.startswith('P1T')]
   if tag=='NOACTION':
    # 动作代次未建立就被拒：不能凭空导出空块或半块。
    assert not frames,'recorded output for a rejected-before-start attempt'
    continue
   assert frames,'empty trace block '+tag
   blocks[tag]=trace_reports(match.group(2).encode())
  assert set(blocks)=={'NORMAL','TRIP','TRIPV','STARTFAIL','STOP','BREAK','NORECOVERY'},sorted(blocks)
  for tag in ('NORMAL','TRIP','TRIPV','STARTFAIL','STOP','BREAK','NORECOVERY'):
   assert len(blocks[tag])==1,tag
  normal=blocks['NORMAL'][0]
  assert normal['complete'] and not normal['overflow'] and normal['post']==8,normal['header']
  assert normal['stages']==[0,1,2,3],normal['stages']
  assert normal['header']['pre']==16 and normal['header']['trip']==0xffffffff
  assert normal['header']['frozen'] and normal['header']['freeze_reason']==0xffffffff
  assert sum(1 for s in normal['samples'] if s['stage']==2)>=15
  assert all(s['flags']&1 for s in normal['samples'])
  trip=blocks['TRIP'][0]
  assert trip['complete'] and trip['header']['freeze_reason']==1 and trip['trip_index']!=0xffffffff
  assert trip['samples'][trip['trip_index']]['raw_u']==2157
  tripv=blocks['TRIPV'][0]
  assert tripv['complete'] and tripv['header']['freeze_reason']==2 and tripv['trip_index']!=0xffffffff
  assert tripv['samples'][tripv['trip_index']]['raw_v']==2158
  # 首UEV配置检查拒绝：动作已发起但从未开MOE，仍必须留下STOPPED阶段，恢复样本不得标成动作段。
  startfail=blocks['STARTFAIL'][0]
  assert startfail['complete'] and startfail['stages']==[0,3] and startfail['header']['trip']==0xffffffff
  assert all(sample['flags']&4 or sample['flags']&8 for sample in startfail['samples'])
  assert all(sample['stage']==3 and sample['pins']==0 for sample in startfail['samples'][startfail['header']['pre']:])
  for tag in ('STOP','BREAK'):
   block=blocks[tag][0]
   assert block['complete'] and block['trip_index']==0xffffffff and block['header']['freeze_reason']==0xffffffff
   # Break由硬件清MOE，软件标志仍必须留下STOPPED阶段，恢复样本不得标成50%动作段。
   assert block['stages']==[0,1,2,3],block['stages']
  missing=blocks['NORECOVERY'][0]
  assert not missing['complete'] and missing['post']==0 and missing['header']['frozen']
  assert missing['tail']['actual']==missing['header']['total']
  # 篡改单帧必须被拒绝，不能静默补齐。
  wire=re.search(r'DQ_TRACE_NORMAL_BEGIN\n(.*?)DQ_TRACE_NORMAL_END',full_output,re.S)[1]
  # 通用采集不能把原始记录帧当成无效行。
  for phase in ('LOW225','HIGH250'):
   assert parse_line(('FW1,01-13-07,DQ_VEC0_'+phase+'\r\n').encode())['mode'].endswith(phase)
  assert parse_line(([line for line in wire.splitlines() if line.startswith('P1T')][0]+'\r\n').encode())['format']=='P1T0'
  damaged=bytearray(wire.encode());spot=damaged.find(b'P1T1,');damaged[spot+6]=ord('F') if damaged[spot+6]!=ord('F') else ord('0')
  try: trace_reports(bytes(damaged)); raise AssertionError('corrupted trace accepted')
  except ValueError: pass
  # 缺帧与错序同样必须拒绝，不能靠补齐通过。
  lines=[line for line in wire.splitlines() if line.startswith('P1T')]
  def rebuild(items):return ('\r\n'.join(items)+'\r\n').encode()
  spot=[i for i,line in enumerate(lines) if line.startswith('P1T1,')][3]
  try: trace_reports(rebuild(lines[:spot]+lines[spot+1:])); raise AssertionError('missing trace frame accepted')
  except ValueError: pass
  try: trace_reports(rebuild(lines[:spot]+[lines[spot+1],lines[spot]]+lines[spot+2:])); raise AssertionError('reordered trace frame accepted')
  except ValueError: pass
  # 格式版本不符必须拒绝，避免旧工具硬解新布局。
  def reframe(line,index,value):
   body=line.split(',')[:-1]
   body[1+index]='%08X'%value
   text=','.join(body)+','
   return text+'%04X'%binascii.crc_hqx(text.encode(),0xffff)
  assert trace_reports(rebuild(lines))[0]['phase_consistent'] is True
  assert trace_reports(rebuild([reframe(lines[0],13,250)]+lines[1:]))[0]['phase_consistent'] is False
  legacy=lines[0].split(',')[:-2];legacy[1]='00000001';body=','.join(legacy)+','
  assert trace_reports(rebuild([body+'%04X'%binascii.crc_hqx(body.encode(),0xffff)]+lines[1:]))[0]['phase_consistent'] is None
  try: trace_reports(rebuild([reframe(lines[0],0,3)]+lines[1:])); raise AssertionError('trace version accepted')
  except ValueError: pass
 (out/(tag+'_report.json')).write_text(json.dumps(parsed,indent=2)+'\n')
 if reference==0 and not vector:
  old=out/'old_order.exe';run([*cpp,'-DTEST_OLD_DQ_ORDER',r/'tests/m1/test_dq_current.cpp',board,*objects,'-Wl,--wrap=motor_runtime_poll_feedback','-lm','-o',old])
  neg=subprocess.run([str(old)],cwd=r,env=env,capture_output=True,text=True,timeout=90)
  assert neg.returncode!=0 and 'status().state==MOTOR_RUNTIME_ACTIVE' in neg.stderr
  evidence['old_main_order_negative']={'head':'ca5bf7a206c63e75685a6624cec80428922412f9','returncode':neg.returncode,'stderr':neg.stderr}
 if reference==0 and not vector:
  old_source=subprocess.check_output(['git','show','dc52c7022a9e5f5e64d3d1230acf9e1193544642:firmware/stm32f103/app/current_loop.c'],cwd=r,text=True)
  pattern=r'void current_loop_abort\(void\)\n\{.*?\n\}'
  old_abort=re.search(pattern,old_source,re.S)[0]
  mutant=out/'old_abort.c';mutant.write_text(re.sub(pattern,lambda _:old_abort,(fw/'app/current_loop.c').read_text(encoding='utf-8'),count=1,flags=re.S),encoding='utf-8')
  mutant_obj=out/'old_abort.o';run([gcc,'-std=c11',*flags,*define,'-c',mutant,'-o',mutant_obj])
  mutant_objects=[mutant_obj if obj.name.endswith('_current_loop.o') else obj for obj in objects]
  old_exe=out/'old_abort.exe';run([*cpp,r/'tests/m1/test_dq_current.cpp',board,*mutant_objects,'-Wl,--wrap=motor_runtime_poll_feedback','-lm','-o',old_exe])
  neg=subprocess.run([str(old_exe)],cwd=r,env=env,capture_output=True,text=True,timeout=90)
  assert neg.returncode and 'count_flow("P1D0,")==1u' in neg.stderr
  evidence['old_abort_negative']={'head':'dc52c7022a9e5f5e64d3d1230acf9e1193544642','returncode':neg.returncode,'stderr':neg.stderr,'old_abort_sha256':hashlib.sha256(old_abort.encode()).hexdigest()}
 print(text.splitlines()[-1])
evidence['result']='PASS';previous.write_text(json.dumps(evidence,indent=2)+'\n')
