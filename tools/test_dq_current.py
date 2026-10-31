"""真实前台电流环/寄存器/生命周期集成；只运行主机，不访问串口或硬件。"""
from pathlib import Path
from dq_report import reports,parse
import subprocess,os,json,hashlib
r=Path(__file__).resolve().parents[1];fw=r/'firmware/stm32f103';out=r/'build/dq_current/host';out.mkdir(parents=True,exist_ok=True)
gcc=Path('D:/mingw64/bin/gcc.exe');env=os.environ.copy();env['PATH']=str(gcc.parent)+os.pathsep+env.get('PATH','')
model=r/'tests/m1/pwm_model';inc=[flag for p in (model,fw/'board',fw/'app',r/'control_core') for flag in ('-I',str(p))]
flags=['-O2','-g','-Wall','-Wextra','-Werror','-pedantic',*inc,'-DBOARD_DQ_CURRENT_LOOP=1','-DBOARD_ALIGNMENT_MODE=2']
sources=[fw/p for p in ('app/motor_runtime.c','app/command.c','app/current_loop.c','board/sample_frame.c')]+[r/'control_core'/p for p in ('feedback.c','sensor_input.c','current_control.c','foc_math.c')]
steps=[]
inputs=[*sources,fw/'board/board_pwm.c',fw/'Core/Src/main.c',fw/'Core/Src/tim.c',fw/'m1_sampling.ioc',fw/'project_sources.json',r/'docs/dq_current_loop.md',r/'tests/m1/test_pwm_gate.cpp',r/'tests/m1/test_dq_current.cpp',Path(__file__),r/'tools/dq_report.py']
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
for reference in (0,40,-40):
 define=['-DBOARD_DQ_REFERENCE_MA='+str(reference)];objects=[]
 for p in sources:
  obj=out/(str(reference)+'_'+p.stem+'.o');run([gcc,'-std=c11',*flags,*define,'-c',p,'-o',obj]);objects.append(obj)
 board=out/(str(reference)+'_board.o');cpp=[gcc.parent/'g++.exe','-std=c++11',*flags,*define]
 run([*cpp,'-include',model/'linkage.h','-x','c++','-c',fw/'board/board_pwm.c','-o',board])
 exe=out/(str(reference)+'.exe');run([*cpp,r/'tests/m1/test_dq_current.cpp',board,*objects,'-lm','-o',exe])
 text=run([exe]);(out/(str(reference)+'_trace.txt')).write_text(text,encoding='utf-8');parsed=reports(text.encode());assert len(parsed)==4
 for report in parsed:
  assert report['plateau_max_q_error_mA']<=6 and report['maximum_abs_id_mA']<=2,report
  assert report['configuration']['ref']==reference
 frame=next(line+'\r\n' for line in text.splitlines() if line.startswith('P1D1,'))
 damaged=bytearray(frame.encode());damaged[8]=ord('F') if damaged[8]!=ord('F') else ord('0')
 try:parse(bytes(damaged));raise AssertionError('corruption accepted')
 except ValueError:pass
 (out/(str(reference)+'_report.json')).write_text(json.dumps(parsed,indent=2)+'\n')
 print(text.splitlines()[-1])
evidence['result']='PASS';previous.write_text(json.dumps(evidence,indent=2)+'\n')
