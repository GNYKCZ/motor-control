"""Build unchanged production C into a host-only MATLAB bridge using bundled Clang.
No global compiler settings, firmware source changes or hardware access.
"""
from pathlib import Path
import subprocess,json,hashlib,os
r=Path(__file__).resolve().parents[3];out=r/'build/research_host';out.mkdir(parents=True,exist_ok=True)
m=Path(os.environ.get('MATLAB_ROOT','D:/Matalab'));clang=m/'toolbox/sldrt/clang/win64/clang.exe';lld=clang.with_name('lld.exe')
inc=m/'toolbox/sldrt/lib/include';evidence={'base_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=r,text=True).strip(),'commands':[],'result':'IN_PROGRESS'}
source=[r/'control_core/current_control.c',r/'control_core/foc_math.c',Path(__file__).parent/'firmware_core_mex.c']
evidence['source_sha256']={str(p.relative_to(r)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source}
def run(cmd):
 p=subprocess.run([str(x) for x in cmd],cwd=r,capture_output=True,text=True);evidence['commands'].append({'argv':[str(x) for x in cmd],'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr});(out/'build_evidence.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
 if p.returncode:raise RuntimeError(p.stderr)
for src,name in zip(source,['current_control','foc_math','bridge']):
 run([clang,'-c','-std=c11','-O2','-DMATLAB_MEX_FILE','-I',inc,'-I',m/'extern/include','-I',r/'control_core',src,'-o',out/(name+'.obj')])
(out/'runtime.c').write_text('int _fltused=0;\n');run([clang,'-c',out/'runtime.c','-o',out/'runtime.obj'])
(out/'ucrt.def').write_text('LIBRARY ucrtbase.dll\nEXPORTS\nsinf\ncosf\nsin\ncos\nhypot\nmemset\nmemcpy\n')
run([lld,'-flavor','link','/lib','/def:'+str(out/'ucrt.def'),'/machine:x64','/out:'+str(out/'ucrt.lib')])
run([lld,'-flavor','link','/dll','/noentry','/export:mexFunction','/out:'+str(out/'firmware_core_mex.mexw64'),*[out/(x+'.obj') for x in ['bridge','current_control','foc_math','runtime']],out/'ucrt.lib','/libpath:'+str(m/'extern/lib/win64/microsoft'),'libmex.lib','libmx.lib'])
evidence['result']='PASS';(out/'build_evidence.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8');print('HOST_C_BRIDGE_BUILD_PASS')
