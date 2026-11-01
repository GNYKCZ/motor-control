"""Build/test research-only portable C contract. Never modifies firmware or accesses hardware."""
from pathlib import Path
import ctypes as ct
import subprocess, os, json, random, math, hashlib
here=Path(__file__).resolve().parent
repo=here.parents[2]
out=repo/'build/research_host'
mat=Path(os.environ.get('MATLAB_ROOT','D:/Matalab'))
clang=mat/'toolbox/sldrt/clang/win64/clang.exe'
if not (out/'ucrt.lib').exists(): subprocess.run(['python',str(here/'build_bridge.py')],check=True)
subprocess.run([str(clang),'-c','-std=c11','-O2','-I',str(mat/'toolbox/sldrt/lib/include'),str(here/'bias_compensation.c'),'-o',str(out/'bias_compensation.obj')],check=True)
subprocess.run([str(clang.with_name('lld.exe')),'-flavor','link','/dll','/noentry','/export:bias_compensate','/out:'+str(out/'bias_compensation.dll'),str(out/'bias_compensation.obj'),str(out/'runtime.obj'),str(out/'ucrt.lib')],check=True)
class Cal(ct.Structure):
    _fields_=[('b0',ct.c_float*2),('k',ct.c_float*2),('err',ct.c_float*2),('dmin',ct.c_float),('dmax',ct.c_float),('vmin',ct.c_float),('vmax',ct.c_float),('id',ct.c_uint32),('model',ct.c_uint8),('cert',ct.c_uint8)]
class In(ct.Structure):
    _fields_=[('adc',ct.c_uint16*2),('duty',ct.c_float*3),('bus',ct.c_float)]
class Out(ct.Structure):
    _fields_=[('current',ct.c_float*3),('upper',ct.c_float*3),('permit',ct.c_uint8)]
fn=ct.CDLL(str(out/'bias_compensation.dll')).bias_compensate
fn.argtypes=[ct.POINTER(Cal),ct.POINTER(In),ct.POINTER(Out)];fn.restype=ct.c_int
cal=Cal((.004,-.003),(.123,.118),(.003,.003),.3,.7,10,14,1,1,1)
rng=random.Random(2818);lsb=3.3/4096;max_error=0;allowed=0
for model in (1,2):
    cal.model=model
    for _ in range(25000):
        inp=In((rng.randrange(1800,2300),rng.randrange(1800,2300)),tuple(rng.uniform(.31,.69) for _ in range(3)),12)
        res=Out();status=fn(cal,inp,res)
        features=list(inp.duty[:2]) if model==1 else [sum(inp.duty)/3]*2
        current=[inp.adc[j]*lsb-1.65-cal.b0[j]-cal.k[j]*features[j] for j in range(2)];current.append(-sum(current))
        bounds=[abs(current[j])+cal.err[j]+lsb/2+2e-6 for j in range(2)]+[abs(current[2])+sum(cal.err)+lsb+4e-6]
        max_error=max(max_error,max(abs(res.upper[j]-bounds[j]) for j in range(3)))
        # Skip decision equivalence only within declared floating-point tolerance of threshold.
        if abs(max(bounds)-128*lsb)>1e-6: assert bool(res.permit)==(max(bounds)<=128*lsb)
        assert status in (0,4);allowed+=res.permit
assert max_error<1e-6
inp=In((2129,2119),(.5,.5,.5),12);res=Out();cal.model=1
assert fn(None,inp,res)==1 and not res.permit
assert fn(cal,None,res)==1 and fn(cal,inp,None)==1
cal.cert=0;assert fn(cal,inp,res)==1;cal.cert=1
cal.model=3;assert fn(cal,inp,res)==1;cal.model=1
cal.k[0]=float('nan');assert fn(cal,inp,res)==1;cal.k[0]=.123
cal.err[0]=-1;assert fn(cal,inp,res)==1;cal.err[0]=.003
inp.adc[0]=0;assert fn(cal,inp,res)==2;inp.adc[0]=4095;assert fn(cal,inp,res)==2;inp.adc[0]=2129
inp.bus=15;assert fn(cal,inp,res)==3;inp.bus=12
inp.duty[0]=float('nan');assert fn(cal,inp,res)==3;inp.duty[0]=.9;assert fn(cal,inp,res)==3
report={'result':'PASS','host_cases':50000,'allowed_cases':allowed,'max_bound_error_A':max_error,'mcu_build':'NOT RUN; research candidate only','production_integration':False,'source_sha256':hashlib.sha256((here/'bias_compensation.c').read_bytes()).hexdigest()}
(here.parent/'results/data/portable_c_checks.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
