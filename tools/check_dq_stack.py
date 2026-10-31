"""实际ARM静态栈/链接布局检查；只重链接同一对象到build，不访问板卡。"""
import argparse,hashlib,html,json,re,struct,subprocess
from pathlib import Path
r=Path(__file__).resolve().parents[1];fw=r/'firmware/stm32f103'
a=argparse.ArgumentParser(description=__doc__);a.add_argument('--mdk',type=Path,default=fw/'MDK-ARM');args=a.parse_args()
mdk=args.mdk.resolve();out=r/'build/dq_current/stack';out.mkdir(parents=True,exist_ok=True)
bin_dir=Path('D:/Keil_v5/ARM/ARMCLANG/bin');link=mdk/'build/m1_sampling.lnp';image=mdk/'build/m1_sampling.axf'
text=link.read_text();assert '--library_type=microlib' in text and '--cpu Cortex-M3' in text
cmd=[str(bin_dir/'armlink.exe'),'--via',str(link),'--output',str(out/'checked.axf'),'--list',str(out/'checked.map'),'--map','--symbols','--callgraph','--info=stack']
p=subprocess.run(cmd,cwd=mdk,capture_output=True,text=True,check=True);(out/'link.log').write_text(p.stdout+p.stderr)
graph=(out/'checked.htm').read_text();plain=html.unescape(re.sub('<[^>]+>',' ',graph));plain=re.sub(r'\s+',' ',plain)
names=['main','USART2_IRQHandler','TIM2_IRQHandler','DMA1_Channel2_IRQHandler','ADC1_2_IRQHandler']
depths={}
for name in names:
 m=re.search(r'(?<![A-Za-z0-9_])'+re.escape(name)+r' \(Thumb,.*?\[Stack\] Max Depth = (\d+)',plain)
 assert m,name;depths[name]=int(m[1])
# 四个可嵌套优先级，另留SysTick、栈对齐与不可恢复异常的保守额外余量。
global_max=int(re.search(r'Maximum Stack Usage = (\d+) bytes',plain)[1])
assert depths['main']==global_max,(depths,global_max)
budget=sum(depths.values())+4*32+64+32+2*32+80
assert budget<=2048,(depths,budget)
map_text=(out/'checked.map').read_text();regions={}
for name in ('RW_IRAM1','RW_STACK_EXTENSION','RW_STACK'):
 m=re.search(r'Execution Region '+name+r' \((?:Exec base|Base): (0x[0-9a-fA-F]+),.*?Size: (0x[0-9a-fA-F]+)',map_text)
 assert m,name;regions[name]=(int(m[1],16),int(m[2],16))
assert regions['RW_IRAM1'][0]==0x20000000 and sum(regions['RW_IRAM1'])<=0x20004800
assert regions['RW_STACK_EXTENSION']==(0x20004800,0x400) and regions['RW_STACK']==(0x20004c00,0x400)
raw=out/'checked.bin';subprocess.run([str(bin_dir/'fromelf.exe'),'--bin','--output',str(raw),str(out/'checked.axf')],check=True)
assert struct.unpack_from('<I',raw.read_bytes())[0]==0x20005000
symbols=subprocess.check_output([str(bin_dir/'fromelf.exe'),'--text','-s',str(image)],text=True)
assert re.search(r'__initial_sp\s+0x20005000',symbols) and '__user_initial_stackheap' not in symbols
inputs=[fw/'MDK-ARM/m1_sampling.sct',fw/'MDK-ARM/startup_stm32f103xb.s',image,link,Path(__file__)]
e=dict(result='PASS',head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=r,text=True).strip(),depths_bytes=depths,conservative_combined_bytes=budget,reserved_stack_bytes=2048,regions=regions,initial_sp='0x20005000',hardware='NOT RUN',limits='Static enabled-path bound plus explicit interrupt allowance; callgraph reports unknown cycles/indirect targets, not a measured peak or arbitrary-fault proof',input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
(out/'evidence.json').write_text(json.dumps(e,indent=2)+'\n');print(json.dumps(e,indent=2))
